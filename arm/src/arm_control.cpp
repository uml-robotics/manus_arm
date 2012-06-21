// =============================================================================
// Name   : arm_control.cpp
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Implements the ROS node "arm_control". This node waits for movement commands,
// and then executes those commands after checking the "arm_monitor" node for
// permission. Code to init the arm is from Abe Shultz.
// Update 6/20/12: Currently bypasses arm_monitor safety checks completely
//                 because they don't work right. May revisit in the future.
// =============================================================================

#include "arm/arm_control.h"
#include <cstdio>

ArmControl::ArmControl()
{
    cmd_server_ = n_.advertiseService("commands",
                                      &ArmControl::cmdServerCallback, this);
    arm_ = ManusArm::instance();
    for (int i = 0; i < 9; i++)
        states_[i] = 0;
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

    while (ros::ok() && !shutdown_)
        ros::spinOnce();
    
    ROS_INFO("ARM control shutting down...");
}

bool ArmControl::cmdServerCallback(arm::command::Request& req,
                                   arm::command::Response& res)
{
    if (req.stop_all)
        stopAll();
    else if (req.quit)
    {
        stopAll();
        shutdown_ = true;
    }
    else if (req.query)
        printStates();
    else
    {
        for (int i = 0; i < 9; i++)
            states_[i] = req.states[i];
        move();
    }

    for (int i = 0; i < 9; i++)
        res.states[i] = states_[i];

    return true;
}

void ArmControl::printStates()
{
    printf("X[%d] Y[%d] Z[%d]\n", states_[1], states_[2], states_[0]);
    printf("Yaw[%d] Pitch[%d] Roll[%d] Grip[%d]\n", states_[3], states_[4],
            states_[5], states_[6]);
    printf("Lift[%d] Speed[%d]\n", states_[7], states_[8]);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_control");
    ArmControl arm_control;
    arm_control.init();
    return 0;
}
