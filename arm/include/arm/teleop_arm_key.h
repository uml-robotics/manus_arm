// =============================================================================
// Name   : teleop_arm_key.h
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Header file for the ROS node "teleop_arm_key".
// =============================================================================

#ifndef TELEOP_ARM_KEY_H_
#define TELEOP_ARM_KEY_H_

#include "ros/ros.h"
#include "arm/constant_move.h"

class TeleopArmKey
{
public:
    TeleopArmKey();
    void init();

private:
    void keyLoop();
    bool getCommand(const char c);
    void print();
      
    ros::NodeHandle n_;
    ros::Publisher cmd_pub_;
    arm::constant_move cmd_;
};

#endif
