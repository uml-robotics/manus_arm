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

void moveDoneCallback() {}

class ArmControl 
{
public:
    ArmControl();
    void init();
    
private:  
    bool cmdServerCallback(arm::command::Request& req,
                           arm::command::Response& res);

    void move()
    {
        arm_->moveConstant(states_.c_array(), &moveDoneCallback);
    }

    void stopAll()
    {
        for (int i = 0; i < 9; i++)
            states_[i] = 0;
        move();
    }

    void executeCommand();
    void printStates();
    
    ros::NodeHandle n_;
    ros::ServiceServer cmd_server_;
    ManusArm* arm_;
    boost::array<int, 9> states_;
    bool shutdown_;
    //ros::ServiceClient arm_health_client_;
    //ros::ServiceClient move_request_client_;
    //std::string last_position_;
    //bool problem_;
    //bool last_problem_;
};

#endif
