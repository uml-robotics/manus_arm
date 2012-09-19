/**
 * \brief ROS node "arm_control"
 * \copyright 2012 University of Massachusetts Lowell
 * \author Jonathan Hasenzahl, Abe Shultz
 *
 * Implements the ROS node "arm_control". This node moves the arm based on
 * commands from teleop nodes.
 */

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
	shutdown_ = false;

	// Initialize subscribers and clients
    cartesian_sub_ = n_.subscribe("cartesian_moves", 1000,
                                  &ArmControl::cartesianMovesCallback, this);
    constant_sub_ = n_.subscribe("constant_moves", 1,
                                 &ArmControl::constantMoveCallback, this);
    constant_time_sub_ = n_.subscribe("constant_move_times", 1,
                                      &ArmControl::constantMoveTimeCallback,
                                      this);
    time_client_ = n_.serviceClient<time_server::time_srv>("time_service");

    // Initialize the ARM
    arm_ = ManusArm::instance();
    try
    {
        arm_->init("can0");
    }
    catch (ArmException& e)
    {
        printf("%s\n", e.what());
        ROS_FATAL("Arm initialization failed");
        return;
    }

    // Move into final position to finish
    for (int i = 0; i < POS_ARR_SZ; i++)
        target_position_[i] = manus_arm::final_position[i];
    speed_ = 2;
    moveCartesian();

    // Start updating thread
    boost::thread move_update(&ArmControl::run, this);

    // Start spinning
    ros::spin();
}

void ArmControl::run()
{
	// Move arm into origin position
    for (int i = 0; i < MOVE_ARR_SZ; i++)
    {
    	cartesian_move_.positions[i] = origin_position[i];
    	cartesian_move_.speeds = 2;
    }
    arm_->moveCartesian(cartesian_move);

	while (!shutdown_)
	{

	}

	// Move arm into final position
    for (int i = 0; i < MOVE_ARR_SZ; i++)
    {
    	cartesian_move_.positions[i] = final_position[i];
    	cartesian_move_.speeds = 2;
    }
    arm_->moveCartesian(cartesian_move);
}

void ArmControl::cartesianMovesCallback(const arm::cartesian_moves::ConstPtr&
                                        cmd)
{
	// TODO: Re-implement this method

    /*time_server::time_srv end_check;
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
    }*/
}

void ArmControl::constantMoveCallback(const arm::constant_move::ConstPtr& cmd)
{
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
                states_[i] = static_cast<int>(cmd->states[i]);
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
            ROS_INFO("Moving...");

            printf("Move start time  : %f\n", cmd->header.stamp.toSec());
            printf("Server time      : %f\n", end_check.response.actual.toSec());
            printf("Delta            : %f\n", (cmd->header.stamp -
                    end_check.response.actual).toSec());
            printf("Duration of move : %f\n", end_check.response.delta.toSec());

            for (int i = 0; i < STATE_ARR_SZ; i++)
                states_[i] = cmd->move.states[i];
            arm_->moveConstant(states_);

            end_check.response.delta.sleep();

            // Stop
            for (int i = 0; i < STATE_ARR_SZ; i++)
                states_[i] = 0;
            arm_->moveConstant(states_);
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
