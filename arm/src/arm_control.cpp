// =============================================================================
// Name   : arm_control.cpp
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Implements the ROS node "arm_control". This node waits for movement commands,
// and then executes those commands after checking the "arm_monitor" node for
// permission. Code to init the arm is from Abe Shultz.
// Update 6/20/12: Safety checking by arm_monitor was broken and is now
//                 unimplemented. May revisit in the future.
// =============================================================================

#include "arm/arm_control.h"
#include <cstdio>
#include <cmath>

void ArmControl::init()
{
    ROS_INFO("ARM control started...");
    cartesian_sub_ = n_.subscribe("cartesian_moves", 1000,
                                  &ArmControl::cartesianMoveCallback, this);
    constant_sub_ = n_.subscribe("constant_moves", 1,
                                 &ArmControl::constantMoveCallback, this);
    arm_ = ManusArm::instance();
    speed_ = manus_arm::speed;
    shutdown_ = false;
    
    try
    {
        arm_->init("can0");
    }
    catch (ArmException& e)
    {
        printf("%s\n", e.what());
        printf("Init failed, bailing\n");
        ros::shutdown();
    }

    // Move into origin position to start
    for (int i = 0; i < POS_ARR_SZ; i++)
        target_position_[i] = manus_arm::origin_position[i];
    moveCartesian();

    while (ros::ok() && !shutdown_)
        ros::spinOnce();

    // Move into final position to finish
    for (int i = 0; i < POS_ARR_SZ; i++)
        target_position_[i] = manus_arm::final_position[i];
    moveCartesian();

    ROS_INFO("ARM control shutting down...");
}

void ArmControl::cartesianMoveCallback(const arm::cartesian_move::ConstPtr& cmd)
{
    ROS_INFO("Moving to X[%10.3f] Y%[%10.3f]", cmd->position[ARM_X],
             cmd->position[ARM_Y]);

    for (unsigned int i = 0; i < POS_ARR_SZ; i++)
        target_position_[i] = cmd->position[i];
    speed_ = cmd->speed;

    moveCartesian();
}

void ArmControl::constantMoveCallback(const arm::constant_move::ConstPtr& cmd)
{
    printf("constantMoveCallback called\n");

    if (cmd->query)
    {
        arm_->getPosition(actual_position_);
        print();
    }
    else
    {
        arm_->getPosition(actual_position_);
        for (int i = 0; i < POS_ARR_SZ; i++)
            target_position_[i] = actual_position_[i];
        moveCartesian();

        if (cmd->quit)
            shutdown_ = true;
        else
        {
            for (int i = 0; i < STATE_ARR_SZ; i++)
                states_[i] = cmd->states[i];
            arm_->moveConstant(states_);
        }
    }
}

void ArmControl::moveCartesian()
{
    // Speed constants for arm
    float Kp[6] = { 5, 5, 5, 0.8, 0.7, 0.6 };

    // Constant speed limits
    int linear_speed_limit[5] = { 10, 30, 50, 70, 90 };

    // Error in position
    float pos_err[3];
    // New speeds
    float speeds[POS_ARR_SZ] = { 0, 0, 0, 0, 0, 0, 0 };

    move_complete_ = false;
    while (!move_complete_)
    {
        // Keep spinning so we can receive new commands
        ros::spinOnce();

        // Get current position
        arm_->getPosition(actual_position_);

        // Calculate the error and speeds
        // Currently only calculates for X Y Z
        for (int i = 0; i < 3; i++)
        {
            pos_err[i] = target_position_[i] - actual_position_[i];
            float control_input = Kp[i] * pos_err[i];
            speeds[i] = fabs(control_input) > linear_speed_limit[speed_] ?
                        sign(control_input) * linear_speed_limit[speed_] :
                        control_input;
        }

        arm_->moveCartesian(speeds);
        ros::Duration(0.06).sleep();

        // Assume we are done
        move_complete_ = true;
        for (int ii = 0; ii < 3; ii++)
        {
            if (fabs(pos_err[ii]) > CARTESIAN_SLOP)
            {
                // Still have moving to do
                move_complete_ = false;
                break;
            }
        }
    }

    // Stop the arm
    arm_->moveCartesian(manus_arm::stop);
    ros::Duration(0.06).sleep();
}

void ArmControl::print()
{
    printf("\n");
    for (int i = 0; i < POS_ARR_SZ; i++)
        printf("%d[%.3f]\n", i, actual_position_[i]);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_control");
    ArmControl arm_control;
    return 0;
}
