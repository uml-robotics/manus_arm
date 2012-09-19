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
#include "arm/cartesian_moves.h"
#include "arm/constant_move.h"
#include "arm/constant_move_time.h"
#include "time_server/time_srv.h"
#include "movement_definitions.h"
#include <list>

class ArmControl 
{
public:
    ArmControl() { init(); }
    
private:
    void init();
    void run();
    void cartesianMovesCallback(const arm::cartesian_moves::ConstPtr& cmd);
    void constantMoveCallback(const arm::constant_move::ConstPtr& cmd);
    void constantMoveTimeCallback(const arm::constant_move_time::ConstPtr& cmd);
    void print();
    
    ros::NodeHandle n_;
    ros::Subscriber cartesian_sub_;
    ros::Subscriber constant_sub_;
    ros::Subscriber constant_time_sub_;
    ros::ServiceClient time_client_;
    ManusArm* arm_;
    boost::mutex data_update_;

    CartesianMove cartesian_move_;
    ConstantMove constant_move_;

    bool move_complete_;
    bool shutdown_;
};

#endif
