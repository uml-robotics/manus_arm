// =============================================================================
// Name   : arm_control.h
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Header file for the ROS node "arm_control".
// =============================================================================

#ifndef ARM_CONTROL_H_
#define ARM_CONTROL_H_

#include "ros/ros.h"
#include "arm/ManusArm.hpp"
#include "arm/command.h"
#include <string>

class ArmControl 
{
public:
    ArmControl();
    void init();
    
private:  
    void callback(const arm::command::ConstPtr& c);
    //void checkHealth();
    void executeCommand();
    //bool requestMove();
    void move();
    void stopAll();
    void printStates();
    
    ros::NodeHandle n_;
    ros::Subscriber command_sub_;
    //ros::ServiceClient arm_health_client_;
    //ros::ServiceClient move_request_client_;
    ManusArm* arm_;
    int movement_states_[9];
    int command_;
    //std::string last_position_;
    //bool problem_;
    //bool last_problem_;
    bool shutdown_;
};

#endif
