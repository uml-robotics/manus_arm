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
#include "arm/movement_definitions.h"
#include <cmath>

void TeleopArmDish::init()
{
    ROS_INFO("ARM control via dish running...");
    cmd_pub_ = n_.advertise<arm::cartesian_move>("cartesian_moves", 1000);

    // Wait for subscriber to "cartesian_moves" before subscribing to "cats"
    ROS_INFO("Waiting for subscriber...");
    while (cmd_pub_.getNumSubscribers() < 1 && ros::ok());
    ROS_INFO("Subscriber found. Continuing...");

    cat_sub_ = n_.subscribe("cats", 1000, &TeleopArmDish::callback, this);

    // Wait for a publisher of "cats" before continuing
    ROS_INFO("Waiting for publisher...");
    while (cat_sub_.getNumPublishers() < 1 && ros::ok());
    ROS_INFO("Publisher found. Continuing...");

    // Continue only while there is a publisher of "cats" or the queue isn't
    // empty
    while((cat_sub_.getNumPublishers() > 0 || !queue_.empty()) && ros::ok())
    {
        ros::spinOnce();
        if (!queue_.empty())
            publishCommand();
    }
    ROS_INFO("ARM control via dish shutting down...");
}

void TeleopArmDish::callback(const burst_calc::cat::ConstPtr& c)
{
    queue_.push(*c);

    static bool run_once = true;
    if (run_once)
    {
        timer_ = ros::Time(0);
        offset_ = ros::Time::now() - timer_;
        run_once = false;
    }
}

void TeleopArmDish::publishCommand()
{
    burst_calc::cat cat = queue_.front();
    queue_.pop();

    if (cat.header.stamp > timer_)
    {
        printf("\nEnd of last burst [%7.3fs] Start of new burst [%7.3fs] Sleep [%6.3fs]\n",
               timer_.toSec(), cat.header.stamp.toSec(),
               (cat.header.stamp - timer_).toSec());
        ros::Duration(cat.header.stamp - timer_).sleep();
    }
    else
        ROS_WARN("CAT timestamp should be greater than Timer and is not");

    // Get the average CA for this CAT
    double x = 0;
    double y = 0;
    int size = cat.cas.size();
    for (int i = 0; i < size; i++)
    {
        x += cat.cas[i].x;
        y += cat.cas[i].y;
    }
    x /= size;
    y /= size;

    // The safe range, in arm units (AU), for the ARM to move in an X/Y
    // square is -20000:20000 AU on each axis (40001 AU total). The actual
    // safe range for each individual axis is -28000:28000, but that will
    // shrink depending on the value of the other axis.
    //
    // Each axis of the CAT grid ranges from 1:8, with 1 in the upper left
    // corner. The midpoint of each axis is 4.5 ([1+8]/2) CAT units (CU).
    //
    // We assign the max range of 4.41:4.59 CU on the X CAT axis to the
    // max range on the ARM Y axis (0.19 CU = 40001 AU). To convert CU to
    // AU: ([CU - midpoint] * [-20000 / 0.09]). The resulting value will
    // always be within the ARM safe range as long as each CA is within
    // the max range we specified.
    //
    // We assign the max range of 4.16:4.84 CU on the Y CAT axis to the
    // max range on the ARM Y axis (0.69 CU = 40001 AU). To convert CU to
    // AU: ([CU - midpoint] * [-20000 / 0.34]). The resulting value will
    // always be within the ARM safe range as long as each CA is within
    // the max range we specified.
    arm::cartesian_move cmd;
    cmd.header.stamp = cat.header.stamp;
    cmd.speed = 3;

    cmd.position[ARM_X] = (x - 4.5) * -222222;
    cmd.position[ARM_Y] = (y - 4.5) * -58823;
    cmd.position[ARM_Z] = manus_arm::origin_position[ARM_Z];
    cmd.position[CLAW_YAW] = manus_arm::origin_position[CLAW_YAW];
    cmd.position[CLAW_PITCH] = manus_arm::origin_position[CLAW_PITCH];
    cmd.position[CLAW_ROLL] = manus_arm::origin_position[CLAW_ROLL];
    cmd.position[CLAW_GRIP] = manus_arm::origin_position[CLAW_GRIP];

    //printf("CA  : x[%10.3f] y[%10.3f]\n", x, y);
    //printf("ARM : x[%10.3f] y[%10.3f]\n", cmd.position[ARM_X], cmd.position[ARM_Y]);

    // Some error checking so we don't exceed the bounds of the ARM
    if (fabs(cmd.position[ARM_X] > 20000))
    {
        ROS_WARN("X-axis position (%f) out of bounds", cmd.position[ARM_X]);
        cmd.position[ARM_X] = 0;
    }
    if (fabs(cmd.position[ARM_Y] > 20000))
    {
        ROS_WARN("Y-axis position (%f) out of bounds", cmd.position[ARM_Y]);
        cmd.position[ARM_Y] = 0;
    }

    // Publish the move and wait
    cmd_pub_.publish(cmd);
    ros::Duration burst_time = cat.end - cat.header.stamp;
    printf("Burst duration    [%7.3fs]\n", burst_time.toSec());
    burst_time.sleep();
    timer_ = ros::Time::now() - offset_;
    printf("Calculated end    [%7.3fs] Actual end         [%7.3fs]\n",
           cat.end.toSec(), timer_.toSec());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_arm_dish");
    TeleopArmDish teleop_arm_dish;
    return 0;
}
