/**
 * \brief ROS node "arm_control"
 * \copyright 2012 University of Massachusetts Lowell
 * \author Jonathan Hasenzahl, Abe Shultz
 *
 * Implements the ROS node "arm_control". This node moves the arm based on
 * commands from teleop nodes.
 */

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

	// Move arm into origin position
    for (int i = 0; i < CART_MV_ARR_SZ; i++)
    {
    	cartesian_move_.positions[i] = ORIGIN_POSITION[i];
    	cartesian_move_.speeds[i] = 2;
    }
    arm_->setMoveComplete(false);
    arm_->moveCartesian(cartesian_move_);

    // Main loop
	while (ros::ok() && !shutdown_)
	{
		ros::spinOnce();
	}

	// Move arm into final position only if ROS is still ok
	if (ros::ok())
	{
		for (int i = 0; i < CART_MV_ARR_SZ; i++)
		{
			cartesian_move_.positions[i] = FINAL_POSITION[0][i];
			cartesian_move_.speeds[i] = 2;
		}
		arm_->setMoveComplete(false);
		arm_->moveCartesian(cartesian_move_);

		while (!arm_->isMoveComplete())
		{
			//printf("Not Complete\n");
			ros::Duration(0.06).sleep();
		}
	}

	printf("Arm control shutdown\n");
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
	if (cmd->quit)
	{
		shutdown_ = true;
	}
	else if (cmd->query)
    {
        print();
    }
    else
    {
    	for (int i = 0; i < CONST_MV_ARR_SZ; i++)
    		constant_move_.states[i] = static_cast<int>(cmd->states[i]);
    	for (int i = 0; i < SPD_ARR_SZ; i++)
    		constant_move_.speeds[i] = static_cast<int>(cmd->speeds[i]);
    	arm_->moveConstant(constant_move_);
    }
}

void ArmControl::constantMoveTimeCallback(const arm::constant_move_time::ConstPtr &cmd)
{
	// TODO: Re-implement this method

    /*time_server::time_srv end_check;
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
        ROS_ERROR("Time server is not responding, movement command skipped");*/
}

void ArmControl::print()
{
	printf("\n%s", arm_->getPrintState().c_str());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_control");
    ArmControl arm_control;
    return 0;
}
