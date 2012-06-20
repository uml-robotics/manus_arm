// =============================================================================
// Name   : teleop_arm_dish.cpp
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Implements the ROS node "teleop_arm_dish". This node generates commands for
// the "arm_control" node based on activity in the neuron dish.
// =============================================================================

#include "arm/teleop_arm_dish.h"
#include "arm/command.h"
#include "electrode/cat.h"
#include "arm/arm_commands.h"

TeleopArmDish::TeleopArmDish()
{
    cat_sub_ = n_.subscribe("cats", 1000, &TeleopArmDish::callback, this);
    command_pub_ = n_.advertise<arm::command>("commands", 1);
    loop_rate_ =  new ros::Rate(2);
    ROS_INFO("Dish control started...");
    while(ros::ok())
    {
        ros::spinOnce();
        getCommands();
    }
    ROS_INFO("Dish control shutting down...");
}

void TeleopArmDish::getCommands()
{
    arm::command command;
    if (!queue_.empty())
    {
        electrode::cat cat = queue_.front();
        queue_.pop();
        for (unsigned int i = 1; i < cat.cas.size() && ros::ok(); i++)
        {
            if (cat.cas[i].x - cat.cas[i-1].x > 0)
            {
                printf("Moving left\n");
                command.data = ARM_LEFT;
            }
            else if (cat.cas[i].x - cat.cas[i-1].x < 0)
            {
                printf("Moving right\n");
                command.data = ARM_RIGHT;
            }
            else
            {
                printf("Not changing movement (x)\n");
                command.data = NONE;
            }
            command_pub_.publish(command);

            if (cat.cas[i].x - cat.cas[i-1].x > 0)
            {
                printf("Moving down\n");
                command.data = ARM_DOWN;
            }
            else if (cat.cas[i].x - cat.cas[i-1].x < 0)
            {
                printf("Moving up\n");
                command.data = ARM_UP;
            }
            else
            {
                printf("Not changing movement (y)\n");
                command.data = NONE;
            }
            command_pub_.publish(command);

            loop_rate_->sleep();
        }
    }
    else
    {
        printf("Queue empty, stopping all movement\n");
        command.data = STOP_ALL;
        command_pub_.publish(command);
        loop_rate_->sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_arm_dish");
    TeleopArmDish teleop_arm_dish;
    return 0;
}
