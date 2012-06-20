// =============================================================================
// Name   : arm_monitor.h
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Header file for the ROS node "arm_monitor".
// =============================================================================

#ifndef ARM_MONITOR_H_
#define ARM_MONITOR_H_

#include "ros/ros.h"
#include "arm/ArmHealth.h"
#include "arm/MoveRequest.h"
#include "arm/arm_commands.h"
#include <string>

enum { Z, X, Y, YAW, PITCH, ROLL, GRIP, SIZE };

const double MAX_STATE[7] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 7400.0 };
const double MIN_STATE[7] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 11000.0 };

class ArmMonitor
{
public:
    ArmMonitor();
    void init();
    
private:
    bool armHealthCallback(arm::ArmHealth::Request &request, 
                           arm::ArmHealth::Response &response);
    bool moveRequestCallback(arm::MoveRequest::Request &request,
                             arm::MoveRequest::Response &response);
    void updateProblemCode();
    void parse(std::string s);
    double deltaGrip();
    void updateMinMax(); // For debugging
    void print(); // For debugging

    ros::NodeHandle n_;
    ros::ServiceServer arm_health_service_;
    ros::ServiceServer move_request_service_;
    double state_[SIZE];
    double origin_[SIZE];
    double min_state_[SIZE];
    double max_state_[SIZE];
    double last_grip_state_;
    int problem_code_;
    bool run_once_;
    bool shutdown_;
};

#endif
