// =============================================================================
// Name     : arm_control.h
// Copyright: 2012 University of Massachusetts Lowell
// Author   : Jonathan Hasenzahl, Abe Shultz
// =============================================================================

#include "arm/arm_control.h"
#include <cstdio>
#include <cmath>

/*!
 * \brief Default constructor
 *
 * Calls the init and run methods.
 */
ArmControl::ArmControl()
{
	init();
	run();
}

/*!
 * \brief Initializes the node
 *
 * There are two functions of the init method: it first subscribes to the
 * necessary ROS topics to receive movement commands, and it then initializes
 * the arm hardware.
 */
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
}

/*!
 * \brief Runs the node in a loop until a shutdown message is received
 *
 * Before the main loop, the arm is moved into its origin position. The method
 * then enters a spin loop until the shutdown flag is set to true (via a ROS
 * message) or Ctrl+C is pressed. After the loop exits, the arm is moved into
 * its final position.
 */
void ArmControl::run()
{
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
			cartesian_move_.positions[i] = FINAL_POSITION[1][i];
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

/*!
 * \brief Callback for cartesian movement messages
 *
 * This method is called automatically when the node receives a cartesian
 * movement message. The received message contains an array of cartesian moves
 * and an ending time. The time server is polled to compare actual time to the
 * ending time, and the arm will be moved, in sequence, according to the array
 * of moves while time is remaining. If there is no time remaining, further
 * moves are not executed and the callback exits.
 *
 * \param cmd the received message
 */
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

            // Check for a current move and stop if necessary
            if (!arm_->isMoveComplete()) arm_->setMoveComplete(true);

            if (end_check.response.delta > ros::Duration(-0.2))
            {
                for (unsigned int j = 0; j < CART_MV_ARR_SZ; j++)
                {
                    cartesian_move_.positions[j] = cmd->moves[i].positions[j];
                    cartesian_move_.speeds[j] = cmd->moves[i].speeds[j];
                }
                arm_->setMoveComplete(false);
                arm_->moveCartesian(cartesian_move_);
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

/*!
 * \brief Callback for constant movement messages
 *
 * This method is called automatically when the node receives a constant
 * movement message. This message usually used for direct user control, such as
 * operation with a keyboard. The received message contains position and speed
 * data, and the callback will move the arm appropriately. The message also
 * contains a query flag (the method will print movement data to the screen) and
 * a quit flag (the method set the shutdown flag to true, causing the node
 * to shutdown).
 *
 * \param cmd the received message
 */
void ArmControl::constantMoveCallback(const arm::constant_move::ConstPtr& cmd)
{
	if (cmd->quit)
	{
		shutdown_ = true;
	}
	else if (cmd->query)
    {
		printf("\n%s", arm_->getPrintState().c_str());
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

/*!
 * \brief Callback for constant movement by time messages
 *
 * This method is called automatically when the node receives a cartesian
 * movement by time message. This method is similar in operation to the
 * cartesian movement callback, instead using constant movements.
 *
 * \param cmd the received message
 */
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

            for (int i = 0; i < CONST_MV_ARR_SZ; i++)
            {
                constant_move_.states[i] = cmd->move.states[i];
                if (i < SPD_ARR_SZ)
                	constant_move_.speeds[i] = cmd->move.speeds[i];
            }
            arm_->moveConstant(constant_move_);

            end_check.response.delta.sleep();

            // Stop
            for (int i = 0; i < CONST_MV_ARR_SZ; i++)
				constant_move_.states[i] = 0;
            arm_->moveConstant(constant_move_);
        }
        else
            ROS_ERROR("This movement would have started after its ending time");
    }
    else
        ROS_ERROR("Time server is not responding, movement command skipped");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_control");
    ArmControl arm_control;
    return 0;
}
