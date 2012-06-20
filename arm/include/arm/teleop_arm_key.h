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

class TeleopArm
{
public:
    TeleopArm();
    void init();

private:
    void keyLoop();
    int8_t getCommand(char c);
      
    ros::NodeHandle n_;
    ros::Publisher command_pub_;
};

#endif
