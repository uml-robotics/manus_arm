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

namespace manus_arm
{
void cartesianMoveDoneCallback()
{
    //printf("cartesianMoveDoneCallback called\n");
    done_moving = true;
}

void constantMoveDoneCallback() {}
}

void ArmControl::init()
{
    ROS_INFO("ARM control started...");
    cartesian_sub_ = n_.subscribe("cartesian_moves", 1000,
                                  &ArmControl::cartesianMoveCallback, this);
    constant_sub_ = n_.subscribe("constant_moves", 1,
                                 &ArmControl::constantMoveCallback, this);
    arm_ = ManusArm::instance();
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
    arm_->moveCartesian(manus_arm::origin_position, manus_arm::STD_SPEED,
                        &manus_arm::cartesianMoveDoneCallback);
    manus_arm::done_moving = false;
    while (!manus_arm::done_moving && ros::ok())
        ros::spinOnce();

    // Range of motion test
    //moveSquare();

    while (ros::ok() && !shutdown_)
    {
        ros::spinOnce();
        if (!queue_.empty())
            moveCartesian();
    }

    // Move into final position to finish
    arm_->moveCartesian(manus_arm::final_position, manus_arm::STD_SPEED,
                        &manus_arm::cartesianMoveDoneCallback);
    manus_arm::done_moving = false;
    while (!manus_arm::done_moving && ros::ok())
        ros::spinOnce();

    ROS_INFO("ARM control shutting down...");
}

void ArmControl::cartesianMoveCallback(const arm::cartesian_move::ConstPtr& cmd)
{
    printf("cartesianMoveCallback called\n");
    // Stop the the ARM if it is still working on its last queue
    manus_arm::done_moving = true;
    printf("queue size before: %u", queue_.size());
    queue_.clear();
    printf(" cleared %u", queue_.size());

    for (unsigned int i = 0; i < cmd->queue.size(); i++)
        queue_.push_back(cmd->queue[i]);
    speed_ = cmd->speed;
    printf(" after %u\n", queue_.size());
}

void ArmControl::constantMoveCallback(const arm::constant_move::ConstPtr& cmd)
{
    if (cmd->query)
    {
        arm_->getPosition(position_);
        print();
    }
    else
    {
        for (int i = 0; i < STATE_ARR_SZ; i++)
            states_[i] = cmd->states[i];
        moveConstant();
        if (cmd->quit)
        {
            //if (!manus_arm::done_moving)
            //    manus_arm::done_moving = true;
            shutdown_ = true;
        }
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
/*
void ArmControl::moveCartesian()
{
    //Speed constants for arm
    float Kp[6] =
    { 5, 5, 5, 0.8, 0.7, 0.6 };

    //Constant speed limits
    int linear_speed_limit[5] =
    { 10, 30, 50, 70, 90 };
    int angular_speed_limit[5] =
    { 1, 3, 5, 7, 9 };
    /* Which speed limits to use. 4 is full, kate's work used 2 when approaching user


    //error in position
    float pos_err[6];
    //previous position
    float prev_pos[6];
    //new speeds
    float newSpeeds[6];

    bool moveComplete = false;
    while (!moveComplete)
    {
        //get current position
        {
            boost::mutex::scoped_lock lock(stateMutex);
            for (int i = 0; i < 6; i++)
                prev_pos[i] = currState.jointPositions[i];
        }

        //Calculate the error and speeds
        register float tmp1, tmp2;
        for (int i = 0; i < 3; i++) // Currently only calculates for X Y Z
        {
            if (i == 5) // roll -180 ~ 180 : linear scaling
            {
                {
                    tmp1 = (180.0f - target_position[i]) - (180.0f - prev_pos[i]);
                }

                if (tmp1 >= 0.0f)
                    if (tmp1 <= 180.0f)
                        tmp2 = tmp1;
                    else
                        tmp2 = tmp1 - 360.0f;
                else if (tmp1 > -180.0f)
                    tmp2 = tmp1;
                else
                    tmp2 = 360.0f + tmp1;

                pos_err[i] = tmp2;
            }
            else
                pos_err[i] = target_position[i] - (prev_pos[i]);

            float control_input = Kp[i] * pos_err[i];

            if (i < 3)
                newSpeeds[i] = (fabs(control_input) > linear_speed_limit[speed_mode]) ? sign(control_input) * linear_speed_limit[speed_mode] : control_input;
            else
                newSpeeds[i] = (fabs(control_input) > angular_speed_limit[speed_mode]) ? sign(control_input) * angular_speed_limit[speed_mode] : control_input;
        }


        struct can_frame move;
        setCbox(CBOX_1_CARTESIAN, &move);
        move.can_dlc = 8;
        move.data[LIFT] = 0; //Lift unit
        move.data[Z] = newSpeeds[ARM_Z];
        move.data[X] = newSpeeds[ARM_X];
        move.data[Y] = newSpeeds[ARM_Y];
        move.data[YAW] = 0; //newSpeeds[CLAW_YAW]; <-- only moves X Y Z
        move.data[PITCH] = 0; //newSpeeds[CLAW_PITCH]; <-- only moves X Y Z
        move.data[ROLL] = 0; //newSpeeds[CLAW_ROLL]; <-- only moves X Y Z
        move.data[GRIP] = 0; //Gripper open/close

        enqueueFrame(move);

        //wait 60 msec
        boost::this_thread::sleep(boost::posix_time::milliseconds(60));

        //Assume we are done
        moveComplete = true;
        for (int ii = 0; ii < 3; ii++) // Currently only calculates for X Y Z
        {
            if (fabs(pos_err[ii]) > CARTESIAN_SLOP)
            {
                //still have moving to do
                moveComplete = false;
                break;
            }
        }
    }

    //Stop the arm, otherwise it continues with whatever speeds it had when the move was done
    struct can_frame move;
    setCbox(CBOX_1_CARTESIAN, &move);
    move.can_dlc = 8;
    move.data[LIFT] = 0;
    move.data[X] = 0;
    move.data[Y] = 0;
    move.data[Z] = 0;
    move.data[YAW] = 0;
    move.data[PITCH] = 0;
    move.data[ROLL] = 0;
    move.data[GRIP] = 0; //Gripper open/close

    enqueueFrame(move);

    //wait 60 msec
    boost::this_thread::sleep(boost::posix_time::milliseconds(60));

}
*/
void ArmControl::moveConstant()
{
    arm_->moveConstant(states_, &manus_arm::constantMoveDoneCallback);
}

void ArmControl::moveSquare()
{
    float square[POS_ARR_SZ];
    for (int i = 0; i < POS_ARR_SZ; i++)
        square[i] = manus_arm::origin_position[i];
    square[ARM_X] = 20000;
    square[ARM_Y] = 20000;
    arm_->moveCartesian(square, manus_arm::STD_SPEED,
                        &manus_arm::cartesianMoveDoneCallback);
    manus_arm::done_moving = false;
    while (!manus_arm::done_moving && ros::ok())
        ros::spinOnce();

    square[ARM_X] = -20000;
    arm_->moveCartesian(square, manus_arm::STD_SPEED,
                        &manus_arm::cartesianMoveDoneCallback);
    manus_arm::done_moving = false;
    while (!manus_arm::done_moving && ros::ok())
        ros::spinOnce();

    square[ARM_Y] = -20000;
    arm_->moveCartesian(square, manus_arm::STD_SPEED,
                        &manus_arm::cartesianMoveDoneCallback);
    manus_arm::done_moving = false;
    while (!manus_arm::done_moving && ros::ok())
        ros::spinOnce();

    square[ARM_X] = 20000;
    arm_->moveCartesian(square, manus_arm::STD_SPEED,
                        &manus_arm::cartesianMoveDoneCallback);
    manus_arm::done_moving = false;
    while (!manus_arm::done_moving && ros::ok())
        ros::spinOnce();

    square[ARM_Y] = 20000;
    arm_->moveCartesian(square, manus_arm::STD_SPEED,
                        &manus_arm::cartesianMoveDoneCallback);
    manus_arm::done_moving = false;
    while (!manus_arm::done_moving && ros::ok())
        ros::spinOnce();
}

void ArmControl::print()
{
    printf("\n");
    for (int i = 0; i < POS_ARR_SZ; i++)
        printf("%d[%.0f]\n", i, position_[i]);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_control");
    ArmControl arm_control;
    return 0;
}
