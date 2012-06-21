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

namespace manus_arm
{
bool done_moving;
const float origin[7] = { 15000.0f, -5000.0f, 18000.0f, 0.0f, 0.0f, 0.0f,
                               -10000.0f };
}

void cartesianMoveDoneCallback() { manus_arm::done_moving = true; }
void constantMoveDoneCallback() {}

class ArmControl 
{
public:
    ArmControl();
    void init();
    
private:  
    bool cmdServerCallback(arm::command::Request& req,
                           arm::command::Response& res);

    void moveConstant()
    {
        arm_->moveConstant(states_.c_array(), &constantMoveDoneCallback);
    }

    void moveCartesian()
    {
        arm_->moveCartesian(position_, 2, &cartesianMoveDoneCallback);
        manus_arm::done_moving = false;
        while (!manus_arm::done_moving && ros::ok())
            ros::spinOnce();
        updatePosition();
    }

    void stopAll()
    {
        // Set everything to 0 except speed
        for (int i = 0; i < 8; i++)
            states_[i] = 0;
        moveConstant();
    }

    void printStates();
    void updatePosition() { arm_->getPosition(position_); }
    void printPosition()
    {
        for (int i = 0; i < 7; i++)
            printf("%d[%.0f]\n", i, position_[i]);
    }
    
    ros::NodeHandle n_;
    ros::ServiceServer cmd_server_;
    ManusArm* arm_;
    boost::array<int, 9> states_;
    float position_[7];
    bool shutdown_;
};

#endif
