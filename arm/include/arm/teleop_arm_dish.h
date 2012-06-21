// =============================================================================
// Name   : teleop_arm_dish.h
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Header file for the ROS node "teleop_arm_dish".
// =============================================================================

#ifndef TELEOP_ARM_DISH_H_
#define TELEOP_ARM_DISH_H_

#include "ros/ros.h"
#include "electrode/cat.h"
#include "arm/command.h"
#include <queue>

class TeleopArmDish
{
public:
    TeleopArmDish();
    void init();

private:
    void callback(const electrode::cat::ConstPtr& c) { queue_.push(*c); }
    void getCommands();

    ros::NodeHandle n_;
    ros::Subscriber cat_sub_;
    ros::ServiceClient cmd_client_;
    arm::command command_;
    std::queue<electrode::cat> queue_;
};

#endif