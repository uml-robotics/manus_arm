// =============================================================================
// Name   : teleop_arm_dish.cpp
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Implements the ROS node "teleop_arm_dish". This node generates commands for
// the "arm_control" node based on activity in the neuron dish.
// =============================================================================

#include "arm/teleop_arm_dish.h"
#include "arm/cartesian_move.h"
#include "arm/cartesian_moves.h"
#include "time_server/time_srv.h"
#include "arm/movement_definitions.h"
#include <cmath>

#define MIDPOINT 4.5

void TeleopArmDish::init()
{
    time_client_ = n_.serviceClient<time_server::time_srv>("time_service");

    // Get ARM speed parameter
    if (!n_.getParam("arm_speed", speed_))
    {
        ROS_ERROR("Could not load arm_speed parameter, default will be used");
        speed_ = 2;
    }

    // Get ARM safe range parameter
    if (!n_.getParam("arm_safe_range", arm_safe_range_))
    {
        ROS_ERROR("Could not load arm_safe_range parameter, default will be used");
        arm_safe_range_ = 20000.0;
    }

    // Get max range from midpoint parameter
    if (!n_.getParam("max_range_from_midpoint", max_range_from_midpoint_))
    {
        ROS_ERROR("Could not load max_range_from_midpoint parameter, default will be used");
        max_range_from_midpoint_ = 1.0;
    }

    cmd_pub_ = n_.advertise<arm::cartesian_moves>("cartesian_moves", 1000);

    // Wait for subscriber to "cartesian_moves"
    ROS_INFO("Waiting for subscriber...");
    while (cmd_pub_.getNumSubscribers() < 1 && ros::ok());
    ROS_INFO("Subscriber found. Continuing...");

    cat_sub_ = n_.subscribe("cats", 1000, &TeleopArmDish::callback, this);

    // Wait for a publisher of "cats"
    ROS_INFO("Waiting for publisher...");
    while (cat_sub_.getNumPublishers() < 1 && ros::ok());
    ROS_INFO("Publisher found. Continuing...");

    // Main loop
    while(ros::ok())
    {
        ros::spinOnce();
        if (!queue_.empty())
            publishCommand();
    }
}

void TeleopArmDish::callback(const burst_calc::cat::ConstPtr& c)
{
    queue_.push(*c);
}

void TeleopArmDish::publishCommand()
{
    burst_calc::cat cat = queue_.front();
    queue_.pop();
    time_server::time_srv cat_start;
    cat_start.request.target = cat.header.stamp;

    if (time_client_.call(cat_start))
    {
        printf("CAT start time : %f\n", cat.header.stamp.toSec());
        printf("Server time    : %f\n", cat_start.response.actual.toSec());
        printf("Delta          : %f\n", cat_start.response.delta.toSec());

        // The delta shouldn't be negative (server time is <= CAT time)
        if (cat_start.response.delta >= ros::Duration(0))
        {
            ROS_INFO("Sleeping for %.3fs", cat_start.response.delta.toSec());
            cat_start.response.delta.sleep();

            //
            arm::cartesian_moves cmd;
            cmd.header.stamp = cat.header.stamp;
            cmd.end = cat.end;
            int size = cat.cas.size();
            for (int i = 0; i < size; i++)
            {
                arm::cartesian_move move;
                move.header.stamp = cat.header.stamp;
                move.speed = speed_;

                // Right now only X/Y movement is implemented
                move.position[ARM_X] = getArmCoord(cat.cas[i].x);
                move.position[ARM_Y] = getArmCoord(cat.cas[i].y);
                move.position[ARM_Z] = manus_arm::origin_position[ARM_Z];
                move.position[CLAW_YAW] = manus_arm::origin_position[CLAW_YAW];
                move.position[CLAW_PITCH] = manus_arm::origin_position[CLAW_PITCH];
                move.position[CLAW_ROLL] = manus_arm::origin_position[CLAW_ROLL];
                move.position[CLAW_GRIP] = manus_arm::origin_position[CLAW_GRIP];

                //printf("CA  : x[%10.3f] y[%10.3f]\n", x, y);
                //printf("ARM : x[%10.3f] y[%10.3f]\n", cmd.position[ARM_X], cmd.position[ARM_Y]);

                // Some error checking so we don't exceed the bounds of the ARM
                if (fabs(move.position[ARM_X] > 20000))
                {
                    ROS_ERROR("X-axis position (%f) out of bounds", move.position[ARM_X]);
                    move.position[ARM_X] = 0;
                }
                if (fabs(move.position[ARM_Y] > 20000))
                {
                    ROS_ERROR("Y-axis position (%f) out of bounds", move.position[ARM_Y]);
                    move.position[ARM_Y] = 0;
                }

                cmd.moves.push_back(move);
            }

            // Publish the move and wait for the duration of the burst
            cmd_pub_.publish(cmd);
            ROS_INFO("Command published, sleeping for %.3fs",
                     (cat.end - cat.header.stamp).toSec());
            (cat.end - cat.header.stamp).sleep();

            time_server::time_srv cat_end;
            cat_end.request.target = cat.end;
            time_client_.call(cat_end);
            printf("CAT end time : %f\n", cat.end.toSec());
            printf("Server time  : %f\n", cat_end.response.actual.toSec());
            printf("Delta        : %f\n", cat_end.response.delta.toSec());
        }
        else
            ROS_ERROR("CAT start time is behind server time, no command will be issued");
    }
    else
        ROS_ERROR("Time server is not responding, no command will be issued");
}

double TeleopArmDish::getArmCoord(double coord)
{
    // arm_safe_range_:
    //     The safe range, in ARM units (AU), for the ARM to move in an X/Y
    //     square is -20000:20000 AU on each axis (40001 AU total). The actual
    //     safe range for each individual axis is -28000:28000, but that will
    //     shrink depending on the value of the other axis.
    //
    // max_range_from_midpoint_:
    //     Each axis of the CAT grid ranges from 1:8, with 1 in the upper left
    //     corner. The midpoint of each axis is 4.5 ([1+8]/2) CAT units (CU).
    //     The max is range is the farthest that the CAT strays from the
    //     midpoint in either direction on either axis.
    //
    // The returned value will always be within the ARM safe range as long as
    // each CA is within the max range we specified.
    return (coord - MIDPOINT) * -(arm_safe_range_ / max_range_from_midpoint_);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_arm_dish");
    TeleopArmDish teleop_arm_dish;
    return 0;
}
