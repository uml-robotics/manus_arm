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
#include "arm/command.h"

class TeleopArmKey
{
public:
    TeleopArmKey();
    void init();

private:
    void keyLoop();
    bool getCommand(const char c);
      
    ros::NodeHandle n_;
    ros::ServiceClient cmd_client_;
    arm::command command_;
};

#endif
