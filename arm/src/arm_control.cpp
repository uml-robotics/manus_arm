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

void moveDoneCallback() { printf("moveDoneCallback called\n"); }

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
    {
        ros::spinOnce();
        if (!queue_.empty())
        {
            for (int i = 0; i < POS_ARR_SZ; i++)
                target_position_[i] = queue_.front().positions[i];
            moveCartesian();
            queue_.pop_front();
        }
    }

    // Move into final position to finish
    for (int i = 0; i < POS_ARR_SZ; i++)
        target_position_[i] = manus_arm::final_position[i];
    moveCartesian();

    ROS_INFO("ARM control shutting down...");
}

void ArmControl::cartesianMoveCallback(const arm::cartesian_move::ConstPtr& cmd)
{
    printf("cartesianMoveCallback called\n");
    // Interrupt the ARM if it is still working on its last queue
    move_complete_ = true;
    queue_.clear();

    for (unsigned int i = 0; i < cmd->queue.size(); i++)
        queue_.push_back(cmd->queue[i]);
    speed_ = cmd->speed;
}

void ArmControl::constantMoveCallback(const arm::constant_move::ConstPtr& cmd)
{
    printf("constantMoveCallback called\n");
    move_complete_ = true;
    if (cmd->query)
    {
        arm_->getPosition(actual_position_);
        print();
    }
    else
    {
        for (int i = 0; i < STATE_ARR_SZ; i++)
            states_[i] = cmd->states[i];
        arm_->moveConstant(states_, &moveDoneCallback);
        if (cmd->quit)
            shutdown_ = true;
    }
}

/*void ArmControl::moveCartesian()
{
    arm::position p = queue_.front();
    queue_.pop_front();
    arm_->moveCartesian(p.positions.c_array(), speed_,
                        &manus_arm::cartesianMoveDoneCallback);
    printf("Moving to X[%.0f] Y[%.0f]\n", p.positions[ARM_X],
           p.positions[ARM_Y]);
    printf("Z[%.0f] Y[%.0f] P[%.0f] R[%.0f] G[%.0f] S[%d]\n",
           p.positions[ARM_Z], p.positions[CLAW_YAW], p.positions[CLAW_ROLL],
           p.positions[CLAW_GRIP], speed_);
    manus_arm::done_moving = false;
    while (!manus_arm::done_moving && ros::ok())
        ros::spinOnce();
    arm_->getPosition(position_);
}*/

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
            //printf("%d: prev: %+10.3f err %+10.3f ctrl %+10.3f spd %+10.3f\n",
            //       i, actual_position_[i], pos_err[i], control_input, speeds[i]);
        }

        /*
        std::cout << "Target Position: ";
        for (int ii = 0; ii < 6; ii++)
        {
            std::cout << target_position_[ii] << ", ";
        }
        std::cout << std::endl;

        std::cout << "Position: ";
        for (int ii = 0; ii < 6; ii++)
        {
            std::cout << actual_position_[ii] << ", ";
        }
        std::cout << std::endl;
        std::cout << "Error: ";
        for (int ii = 0; ii < 3; ii++)
        {
            std::cout << pos_err[ii] << ", ";
        }
        std::cout << std::endl;
        std::cout << "New speeds: ";
        for (int ii = 0; ii < 3; ii++)
        {
            std::cout << speeds[ii] << ", ";
        }
        std::cout << std::endl << std::endl;
        */

        arm_->moveCartesian(speeds, &moveDoneCallback);
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
    arm_->moveCartesian(manus_arm::stop, &moveDoneCallback);
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
