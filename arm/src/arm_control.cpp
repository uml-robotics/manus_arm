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

ArmControl::ArmControl()
{
    cartesian_sub_ = n_.subscribe("cartesian_moves", 1000,
                                  &ArmControl::cartesianMoveCallback, this);
    constant_sub_ = n_.subscribe("constant_moves", 1,
                                 &ArmControl::constantMoveCallback, this);
    arm_ = ManusArm::instance();
    shutdown_ = false;
}

void ArmControl::init()
{
    ROS_INFO("ARM control started...");    
    
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

    arm_->moveCartesian(manus_arm::origin_position, manus_arm::STD_SPEED,
                        &cartesianMoveDoneCallback);

    while (ros::ok() && !shutdown_)
    {
        ros::spinOnce();
        if (!queue_.empty())
            moveCartesian();
    }

    ROS_INFO("ARM control shutting down...");
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
            shutdown_ = true;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_control");
    ArmControl arm_control;
    arm_control.init();
    return 0;
}
