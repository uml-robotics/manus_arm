// =============================================================================
// Name     : arm_control.h
// Copyright: 2012 University of Massachusetts Lowell
// Author   : Jonathan Hasenzahl
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

/*!
 * \brief Implements ROS node "arm_control"
 *
 * This node moves the arm based on commands from teleop nodes.
 *
 * \copyright 2012 University of Massachusetts Lowell
 * \author Jonathan Hasenzahl
 */
class ArmControl 
{
public:
	/*!
	 * \brief Default constructor
	 *
	 * Initializes the arm hardware and subscribes to movement topics.
	 */
    ArmControl() { init(); }
    
private:
    void init();
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

    CartesianMove cartesian_move_;
    ConstantMove constant_move_;

    bool move_complete_;
    bool shutdown_;
};

#endif
