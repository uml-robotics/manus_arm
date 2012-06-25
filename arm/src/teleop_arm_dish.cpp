// =============================================================================
// Name   : teleop_arm_dish.cpp
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Implements the ROS node "teleop_arm_dish". This node generates commands for
// the "arm_control" node based on activity in the neuron dish.
// =============================================================================

#include "arm/teleop_arm_dish.h"
#include "electrode/cat.h"
#include "arm/movement_definitions.h"
#include <cmath>

TeleopArmDish::TeleopArmDish()
{
    cat_sub_ = n_.subscribe("cats", 1000, &TeleopArmDish::callback, this);
    cmd_pub_ = n_.advertise<arm::cartesian_move>("cartesian_moves", 1000);
    cmd_.positions[ARM_Z] = manus_arm::origin_position[ARM_Z];
    cmd_.positions[CLAW_YAW] = manus_arm::origin_position[CLAW_YAW];
    cmd_.positions[CLAW_PITCH] = manus_arm::origin_position[CLAW_PITCH];
    cmd_.positions[CLAW_ROLL] = manus_arm::origin_position[CLAW_ROLL];
    cmd_.positions[CLAW_GRIP] = manus_arm::origin_position[CLAW_GRIP];
    cmd_.speed = 2;
}

void TeleopArmDish::init()
{
    ROS_INFO("Dish control started...");
    while (cmd_pub_.getNumSubscribers() < 1 && ros::ok());
    while(ros::ok())
    {
        ros::spinOnce();
        if (!queue_.empty())
            getCommand();
    }
    ROS_INFO("Dish control shutting down...");
}

void TeleopArmDish::getCommand()
{
    electrode::cat cat = queue_.front();
    queue_.pop();

    // For each CA in the CAT, calculate and publish Cartesian command
    for (unsigned int i = 0; i < cat.cas.size(); i++)
    {
        // For a range of 4 to 5 on the electrode grid, 1 dish unit =
        // 56001 arm units. 4.5 is the midpoint and that is what is subtracted
        // from each CA. The result is multiplied by -56000 to get arm units.
        // This ensures that as long as the CA is between 4 and 5, it will never
        // exceed -28000 or 28000 arm units.
        cmd_.positions[ARM_X] = (cat.cas[i].x - 4.35) * -112000;
        cmd_.positions[ARM_Y] = (cat.cas[i].y - 4.35) * -112000;
        //printf("x[%f] y[%f]\n", cmd_.positions[ARM_X], cmd_.positions[ARM_Y]);

        // A little error checking so we don't exceed the bounds of the ARM
        if (fabs(cmd_.positions[ARM_X] > 28000))
        {
            ROS_ERROR("X-axis position (%f) out of bounds",
                      cmd_.positions[ARM_X]);
            cmd_.positions[ARM_X] = 0;
        }
        if (fabs(cmd_.positions[ARM_Y] > 28000))
        {
            ROS_ERROR("Y-axis position (%f) out of bounds",
                      cmd_.positions[ARM_Y]);
            cmd_.positions[ARM_Y] = 0;
        }

        cmd_pub_.publish(cmd_);
    }

    // CAT finished, return the ARM to origin for next sequence
    printf("CAT published, returning to origin\n");
    cmd_.positions[ARM_X] = manus_arm::origin_position[ARM_X];
    cmd_.positions[ARM_Y] = manus_arm::origin_position[ARM_Y];
    cmd_pub_.publish(cmd_);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_arm_dish");
    TeleopArmDish teleop_arm_dish;
    teleop_arm_dish.init();
    return 0;
}
