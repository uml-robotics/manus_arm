// =============================================================================
// Name   : teleop_arm_dish.cpp
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Implements the ROS node "teleop_arm_dish". This node generates commands for
// the "arm_control" node based on activity in the neuron dish.
// =============================================================================

#include "arm/teleop_arm_dish.h"
#include "burst_calc/cat.h"
#include "arm/movement_definitions.h"
#include <cmath>

TeleopArmDish::TeleopArmDish()
{
    cmd_pub_ = n_.advertise<arm::cartesian_move>("cartesian_moves", 1000);
    cmd_.positions[ARM_Z] = manus_arm::origin_position[ARM_Z];
    cmd_.positions[CLAW_YAW] = manus_arm::origin_position[CLAW_YAW];
    cmd_.positions[CLAW_PITCH] = manus_arm::origin_position[CLAW_PITCH];
    cmd_.positions[CLAW_ROLL] = manus_arm::origin_position[CLAW_ROLL];
    cmd_.positions[CLAW_GRIP] = manus_arm::origin_position[CLAW_GRIP];
    cmd_.speed = 3;
}

void TeleopArmDish::init()
{
    ROS_INFO("ARM control via dish running...");

    // Wait for subscriber to "cartesian_moves" before subscribing to "cats"
    ROS_INFO("Waiting for subscriber...");
    while (cmd_pub_.getNumSubscribers() < 1 && ros::ok());
    ROS_INFO("Subscriber found. Continuing...");

    cat_sub_ = n_.subscribe("cats", 1000, &TeleopArmDish::callback, this);

    // Wait for a publisher of "cats" before continuing
    ROS_INFO("Waiting for publisher...");
    while (cat_sub_.getNumPublishers() < 1 && ros::ok());
    ROS_INFO("Publisher found. Continuing...");

    // Continue only while there is a publisher of "cats"
    while(cat_sub_.getNumPublishers() > 0 && ros::ok())
    {
        ros::spinOnce();
        if (!queue_.empty())
            getCommand();
    }
    ROS_INFO("ARM control via dish shutting down...");
}

void TeleopArmDish::getCommand()
{
    burst_calc::cat cat = queue_.front();
    queue_.pop();

    // For each CA in the CAT, calculate and publish Cartesian command
    for (unsigned int i = 0; i < cat.cas.size(); i++)
    {
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
        cmd_.positions[ARM_X] = (cat.cas[i].x - 4.5) * -222222;
        cmd_.positions[ARM_Y] = (cat.cas[i].y - 4.5) * -58823;

        /*
        printf("CA  : x[%f] y[%f]\n", cat.cas[i].x, cat.cas[i].y);
        printf("ARM : x[%f] y[%f]\n", cmd_.positions[ARM_X],
                                      cmd_.positions[ARM_Y]);
        */

        // Some error checking so we don't exceed the bounds of the ARM
        if (fabs(cmd_.positions[ARM_X] > 20000))
        {
            ROS_ERROR("X-axis position (%f) out of bounds",
                      cmd_.positions[ARM_X]);
            cmd_.positions[ARM_X] = 0;
        }
        if (fabs(cmd_.positions[ARM_Y] > 20000))
        {
            ROS_ERROR("Y-axis position (%f) out of bounds",
                      cmd_.positions[ARM_Y]);
            cmd_.positions[ARM_Y] = 0;
        }

        cmd_pub_.publish(cmd_);
    }

    // CAT finished, return the ARM to origin for next sequence
    printf("CAT %d.%d published\n", cat.header.stamp.sec,
           cat.header.stamp.nsec);
    cmd_.positions[ARM_X] = manus_arm::origin_position[ARM_X];
    cmd_.positions[ARM_Y] = manus_arm::origin_position[ARM_Y];
    cmd_pub_.publish(cmd_);

    //ros::shutdown(); // <-- only send publish one CAT for testing
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_arm_dish");
    TeleopArmDish teleop_arm_dish;
    teleop_arm_dish.init();
    return 0;
}
