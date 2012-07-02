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
#include "arm/cartesian_move.h"
#include "arm/constant_move.h"
#include "movement_definitions.h"
#include <list>

class ArmControl 
{
public:
    ArmControl() { init(); }
    void init();
    
private:
    void cartesianMoveCallback(const arm::cartesian_move::ConstPtr& cmd);
    void constantMoveCallback(const arm::constant_move::ConstPtr& cmd);
    void moveCartesian();
    void print();
    
    ros::NodeHandle n_;
    ros::Subscriber cartesian_sub_;
    ros::Subscriber constant_sub_;
    ManusArm* arm_;

    int speed_;
    float actual_position_[POS_ARR_SZ];
    float target_position_[POS_ARR_SZ];
    int states_[STATE_ARR_SZ];
    bool move_complete_;
    bool shutdown_;
};

#endif
