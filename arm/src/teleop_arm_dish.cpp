/*
 * teleop_arm_dish.h
 * Copyright 2013 University of Massachusetts Lowell
 */

#include "arm/teleop_arm_dish.h"
#include "arm/cartesian_move.h"
#include "arm/cartesian_moves.h"
#include "arm/constant_move_time.h"
#include "time_server/time_srv.h"
#include "arm/movement_definitions.h"
#include <cmath>

#define MIDPOINT 4.5

/*!
 * \brief Default constructor
 * Calls the init and run methods.
 */
TeleopArmDish::TeleopArmDish()
{
	init();
	run();
}

/*!
 * \brief Initializes the node
 *
 * This method first calls getParam to get ROS server parameters. It then
 * starts the publisher and service client, and waits for a subscriber to its
 * publisher before starting its own subscriber.
 */
void TeleopArmDish::init()
{
    // Get parameters
    getParams();

    // Initialize client and publisher
    //     Cartesian commands or constant move commands: for now you can use one
    //     or the other. Keep one commented out.
    //cmd_pub_ = n_.advertise<arm::cartesian_moves>("cartesian_moves", 1000);
    cmd_pub_ = n_.advertise<arm::constant_move_time>("constant_move_times", 1);
    time_client_ = n_.serviceClient<time_server::time_srv>("time_service");

    // Wait for subscriber
    ROS_INFO("Waiting for subscriber...");
    while (cmd_pub_.getNumSubscribers() < 1 && ros::ok());

    // Initialize subscriber
    cat_sub_ = n_.subscribe("cats", 1000, &TeleopArmDish::callback, this);
}

/*!
 * \brief Gets the ROS server parameters necessary for proper operation.
 */
void TeleopArmDish::getParams()
{
    // Get ARM speed parameter
    if (!n_.getParam("arm_speed", speed_))
    {
        ROS_WARN("Could not load arm_speed parameter, default is 2");
        speed_ = 2;
    }

    // Get ARM safe range parameter
    if (!n_.getParam("arm_safe_range", arm_safe_range_))
    {
        ROS_WARN("Could not load arm_safe_range parameter, default is 20000.0");
        arm_safe_range_ = 20000.0;
    }

    // Get max range from midpoint parameter
    if (!n_.getParam("max_range_from_midpoint", max_range_from_midpoint_))
    {
        ROS_WARN("Could not load max_range_from_midpoint parameter, default is 1.0");
        max_range_from_midpoint_ = 1.0;
    }
}

/*!
 * \brief Runs the node in a loop until Ctrl+C is pressed
 *
 * In the loop, the node spins and publishes any commands that are waiting
 * in the queue, using either publishCartesianMove or publishConstantMove.
 */
void TeleopArmDish::run()
{
    while(ros::ok())
    {
        ros::spinOnce();
        if (!queue_.empty())
        {
            // Use one or the other, keep only one uncommented
            //publishCartesianMove();
            publishConstantMove();
        }
    }
}

/*!
 * \brief Callback for center of trajectory messages
 *
 * This method is called automatically when the node receieves a center of
 * activity trajectory message. It puts the message into the command queue.
 *
 * \param c the received message
 */
void TeleopArmDish::callback(const burst_calc::cat::ConstPtr& c)
{
    queue_.push(*c);
}

/*!
 * \brief Publishes a cartesian movement command
 *
 * This method retrieves a center of activity trajectory (CAT) message from the
 * front of the command queue and creates and publishes a cartesian movement
 * message derived from that CAT.
 */
void TeleopArmDish::publishCartesianMove()
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
            (cat_start.response.delta - ros::Duration(0.1)).sleep();

            //
            arm::cartesian_moves cmd;
            cmd.header.stamp = cat.header.stamp;
            cmd.end = cat.end;
            int size = cat.cas.size();
            for (int i = 0; i < size; i++)
            {
                arm::cartesian_move move;
                move.header.stamp = cat.header.stamp;

                // Currently all axes have the same speed
                move.speeds.fill(static_cast<int8_t>(speed_));

                // Right now only X/Y movement is implemented
                move.positions[X] = getArmCoord(cat.cas[i].x);
                move.positions[Y] = getArmCoord(cat.cas[i].y);
                move.positions[Z] = ORIGIN_POSITION[Z];
                move.positions[YAW] = ORIGIN_POSITION[YAW];
                move.positions[PITCH] = ORIGIN_POSITION[PITCH];
                move.positions[ROLL] = ORIGIN_POSITION[ROLL];
                move.positions[GRIP] = ORIGIN_POSITION[GRIP];

                //printf("CA  : x[%10.3f] y[%10.3f]\n", x, y);
                //printf("ARM : x[%10.3f] y[%10.3f]\n", cmd.position[ARM_X], cmd.position[ARM_Y]);

                // Some error checking so we don't exceed the bounds of the ARM
                if (fabs(move.positions[X] > 20000))
                {
                    ROS_ERROR("X-axis position (%f) out of bounds", move.positions[X]);
                    move.positions[X] = 0;
                }
                if (fabs(move.positions[Y] > 20000))
                {
                    ROS_ERROR("Y-axis position (%f) out of bounds", move.positions[Y]);
                    move.positions[Y] = 0;
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

/*!
 * \brief Publishes a constant movement command
 *
 * This method retrieves a center of activity trajectory (CAT) message from the
 * front of the command queue and creates and publishes a constant movement
 * message derived from that CAT.
 */
void TeleopArmDish::publishConstantMove()
{
    burst_calc::cat cat = queue_.front();
    queue_.pop();
    time_server::time_srv cat_start;
    cat_start.request.target = cat.header.stamp;

    if (time_client_.call(cat_start))
    {
        /*printf("CAT start time : %f\n", cat.header.stamp.toSec());
        printf("Server time    : %f\n", cat_start.response.actual.toSec());
        printf("Delta          : %f\n", cat_start.response.delta.toSec());*/

        // The delta shouldn't be negative (server time is <= CAT time)
        if (cat_start.response.delta >= ros::Duration(0))
        {
            ROS_INFO("SLeeping for %.3fs before publishing command",
                     cat_start.response.delta.toSec());
            cat_start.response.delta.sleep();

            arm::constant_move_time cmd;
            cmd.header.stamp = cat.header.stamp;
            cmd.end = cat.end;
            int size = cat.cas.size();
            double x = 0.0;
            double y = 0.0;

            // Add all of the CA coordinates
            for (int i = 0; i < size; i++)
            {
                x += cat.cas[i].x;
                y += cat.cas[i].y;
            }

            // Get the average coordinates of the CAT
            x /= size;
            y /= size;
            printf("X: %.3f Y: %.3f\n", x, y);

            // Set the applicable movement states.
            // If coordinate < midpoint, movement is forward/left
            // If coordinate > midpoint is negative, movement is backward/right
            if (y - MIDPOINT < 0)
            {
                ROS_INFO("Y is ABOVE the midpoint, moving FORWARD");
                cmd.move.states[X] = ARM_FORWARD;
            }
            else
            {
                ROS_INFO("Y is BELOW the midpoint, moving BACKWARD");
                cmd.move.states[X] = ARM_BACKWARD;
            }

            if (x - MIDPOINT < 0)
            {
                ROS_INFO("X is to the LEFT of the midpoint, moving LEFT");
                cmd.move.states[Y] = ARM_LEFT;
            }
            else
            {
                ROS_INFO("X is to the RIGHT of the midpoint, moving RIGHT");
                cmd.move.states[Y] = ARM_RIGHT;
            }


            // Currently all axes have the same speed
            cmd.move.speeds.fill(static_cast<int8_t>(speed_));

            // Everything else is 0 (doesn't move)
            cmd.move.states[Z] = 0;
            cmd.move.states[YAW] = 0;
            cmd.move.states[PITCH] = 0;
            cmd.move.states[ROLL] = 0;
            cmd.move.states[GRIP] = 0;
            cmd.move.states[LIFT] = 0;

            printf("[%d] [%d] [%d] [%d] [%d] [%d] [%d] [%d]\n",
                   cmd.move.states[X], cmd.move.states[Y],
                   cmd.move.states[Z], cmd.move.states[YAW],
                   cmd.move.states[PITCH], cmd.move.states[ROLL],
                   cmd.move.states[GRIP], cmd.move.states[LIFT]);
            printf("[%d] [%d] [%d] [%d] [%d] [%d] [%d]\n",
                   cmd.move.speeds[X], cmd.move.speeds[Y],
                   cmd.move.speeds[Z], cmd.move.speeds[YAW],
                   cmd.move.speeds[PITCH], cmd.move.speeds[ROLL],
                   cmd.move.speeds[GRIP]);

            // Publish the move and wait for the duration of the burst
            cmd_pub_.publish(cmd);
            ROS_INFO("Sleeping for duration of command (%.3fs)",
                     (cat.end - cat.header.stamp).toSec());
            (cat.end - cat.header.stamp).sleep();

            /*time_server::time_srv cat_end;
            cat_end.request.target = cat.end;
            time_client_.call(cat_end);
            printf("CAT end time : %f\n", cat.end.toSec());
            printf("Server time  : %f\n", cat_end.response.actual.toSec());
            printf("Delta        : %f\n", cat_end.response.delta.toSec());*/
        }
        else
        {
            ROS_ERROR("CAT start time (%.3f) is behind server time (%.3f): no command will be issued",
                      cat_start.request.target.toSec(), cat_start.response.actual.toSec());
        }
    }
    else
        ROS_ERROR("Time server is not responding: no command will be issued");
}

/*!
 * \brief Converts CAT units to ARM units.
 *
 * CAT units range from 1 to 8 on each axis. ARM units are described in
 * ManusArm.hpp and range from approximately -28,000 to 28,000 on each axis.
 *
 * \param coord the coordinate in CAT units
 * \return the coordinate in ARM units
 */
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

/*!
 * \brief Creates an instance of the node
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_arm_dish");
    TeleopArmDish teleop_arm_dish;
    return 0;
}
