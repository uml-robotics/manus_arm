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
#include "burst_calc/cat.h"
#include <queue>

class TeleopArmDish
{
public:
    TeleopArmDish() { init(); }
    void init();

private:
    void callback(const burst_calc::cat::ConstPtr& c) { queue_.push(*c); }
    void publishCommand();

    ros::NodeHandle n_;
    ros::Subscriber cat_sub_;
    ros::Publisher cmd_pub_;
    std::queue<burst_calc::cat> queue_;
};

#endif
