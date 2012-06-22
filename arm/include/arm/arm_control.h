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
#include <queue>
#include <string>

namespace manus_arm
{
bool done_moving;
float ORIGIN_POSITION[] = { 12000, 0, 0, 0, 0, -1500, -10000 };
float FINAL_POSITION[] = { 7000, 0, 8000, 0, 0, 0, -10000 }; // Not working
int STD_SPEED = 2;
}

void cartesianMoveDoneCallback()
{
    manus_arm::done_moving = true;
    printf("cartesianMoveDoneCallback called\n");
}
void constantMoveDoneCallback() {}

class ArmControl 
{
public:
    ArmControl();
    void init();
    
private:
    void cartesianMoveCallback(const arm::cartesian_move::ConstPtr& cmd)
    {
        queue_.push(*cmd);
    }
    void constantMoveCallback(const arm::constant_move::ConstPtr& cmd);

    void moveCartesian()
    {
        arm_->moveCartesian(queue_.front().positions.c_array(),
                            queue_.front().speed, &cartesianMoveDoneCallback);
        manus_arm::done_moving = false;
        while (!manus_arm::done_moving && ros::ok())
            ros::spinOnce();
        arm_->getPosition(position_);
        queue_.pop();
    }

    void moveConstant()
    {
        arm_->moveConstant(states_, &constantMoveDoneCallback);
    }

    void print()
    {
        printf("\n");
        for (int i = 0; i < POS_ARR_SZ; i++)
            printf("%d[%.0f]\n", i, position_[i]);
    }
    
    ros::NodeHandle n_;
    ros::Subscriber cartesian_sub_;
    ros::Subscriber constant_sub_;
    ManusArm* arm_;
    std::queue<arm::cartesian_move> queue_;
    float position_[POS_ARR_SZ];
    int states_[STATE_ARR_SZ];
    int speed_;
    bool shutdown_;
};

#endif
