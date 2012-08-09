// =============================================================================
// Name   : arm_control.cpp
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Implements the ROS node "arm_control". This node moves the arm based on
// commands from teleop nodes. Code to init the arm is from Abe Shultz.
// =============================================================================

#include "arm/arm_control.h"
#include <cstdio>
#include <cmath>

void ArmControl::init()
{
    cartesian_sub_ = n_.subscribe("cartesian_moves", 1000,
                                  &ArmControl::cartesianMovesCallback, this);
    constant_sub_ = n_.subscribe("constant_moves", 1,
                                 &ArmControl::constantMoveCallback, this);
    constant_time_sub_ = n_.subscribe("constant_move_times", 1,
                                      &ArmControl::constantMoveTimeCallback,
                                      this);
    time_client_ = n_.serviceClient<time_server::time_srv>("time_service");
    arm_ = ManusArm::instance();
    shutdown_ = false;
    
    // Get ARM speed parameter
    if (!n_.getParam("arm_speed", speed_))
    {
        ROS_ERROR("Could not load arm_speed parameter, default will be used");
        speed_ = 2;
    }

    // Init the ARM
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
    int temp_speed = speed_;
    speed_ = 2;
    moveCartesian();
    speed_ = temp_speed;

    while (ros::ok() && !shutdown_)
        ros::spinOnce();

    // Move into final position to finish
    for (int i = 0; i < POS_ARR_SZ; i++)
        target_position_[i] = manus_arm::final_position[i];
    speed_ = 2;
    moveCartesian();
}

void ArmControl::cartesianMovesCallback(const arm::cartesian_moves::ConstPtr&
                                        cmd)
{
    time_server::time_srv end_check;
    end_check.request.target = cmd->end;

    for (unsigned int i = 0; i < cmd->moves.size(); i++)
    {
        if (time_client_.call(end_check))
        {
            printf("CAT end time : %f\n", end_check.request.target.toSec());
            printf("CA run time  : %f\n", cmd->moves[i].header.stamp.toSec());
            printf("Server time  : %f\n", end_check.response.actual.toSec());
            printf("Delta        : %f\n", end_check.response.delta.toSec());

            if (end_check.response.delta > ros::Duration(-0.2))
            {
                for (unsigned int j = 0; j < POS_ARR_SZ; j++)
                    target_position_[j] = cmd->moves[i].position[j];
                speed_ = cmd->moves[i].speed;
                moveCartesian();
            }
            else
            {
                ROS_INFO("Out of time, movement sequence over!");
                return;
            }
        }
        else
        {
            ROS_ERROR("Time server is not responding");
            return;
        }
    }
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

void ArmControl::constantMoveTimeCallback(const arm::constant_move_time::ConstPtr &cmd)
{
    time_server::time_srv end_check;
    end_check.request.target = cmd->end;

    if (time_client_.call(end_check))
    {
        if (end_check.response.delta > ros::Duration(0))
        {
            printf("Move start time  : %f\n", cmd->header.stamp.toSec());
            printf("Server time      : %f\n", end_check.response.actual.toSec());
            printf("Delta            : %f\n", (cmd->header.stamp -
                    end_check.response.actual).toSec());
            printf("Duration of move : %f\n", end_check.response.delta.toSec());

            for (int i = 0; i < STATE_ARR_SZ; i++)
                states_[i] = cmd->move.states[i];
            arm_->moveConstant(states_);
            ROS_INFO("Moving...");

            /*if (end_check.response.delta < ros::Duration(0.25))
                ros::Duration(0.25).sleep();
            else if (end_check.response.delta > ros::Duration(0.75))
                ros::Duration(0.75).sleep();
            else*/
                end_check.response.delta.sleep();

            // Stop
            for (int i = 0; i < STATE_ARR_SZ; i++)
                states_[i] = 0;
            arm_->moveConstant(states_);
            ROS_INFO("Movement finished");
        }
        else
            ROS_ERROR("This movement would have started after its ending time");
    }
    else
        ROS_ERROR("Time server is not responding, movement command skipped");
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
