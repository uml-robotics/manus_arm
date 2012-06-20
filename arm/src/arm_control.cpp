// =============================================================================
// Name   : arm_control.cpp
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Implements the ROS node "arm_control". This node waits for movement commands,
// and then executes those commands after checking the "arm_monitor" node for
// permission.
// Update 6/20/12: Currently bypasses arm_monitor safety checks completely
//                 because they don't work right. May revisit in the future.
// =============================================================================

#include "arm/arm_control.h"
#include "arm/ArmHealth.h"
#include "arm/MoveRequest.h"
#include "arm/arm_commands.h"
#include <cstdio>

ArmControl::ArmControl()
{
    command_sub_ = n_.subscribe("commands", 1, &ArmControl::callback, this);
    //arm_health_client_ = n_.serviceClient<arm::ArmHealth>("arm_health");
    //move_request_client_ = n_.serviceClient<arm::MoveRequest>("move_request");
    arm_ = ManusArm::instance();
    for (int i = 0; i < SIZE; i++)
        movement_states_[i] = 0;
    command_ = NONE;
    //problem_ = false;
    //last_problem_ = false;
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

    //last_position_ = arm_->getCsvState();

    while (ros::ok() && !shutdown_)
    {
        ros::spinOnce();
        //checkHealth();
        executeCommand();
        command_ = NONE;        
    }
    
    //arm::ArmHealth shutdown_call;
    //shutdown_call.request.state = "shutdown";
    //arm_health_client_.call(shutdown_call);
    
    ROS_INFO("ARM control shutting down...");
}

/*void ArmControl::checkHealth()
{
    std::string new_position = arm_->getCsvState();
    if (new_position.compare(last_position_) != 0)
    {   
        arm::ArmHealth state_call;
        state_call.request.state = new_position;//arm_->getCsvState();
    
        if (arm_health_client_.call(state_call))
        {
            problem_ = static_cast<bool>(state_call.response.decision);
            if (problem_ && !last_problem_)
            {
                allStop();
                ROS_INFO("Movement halted (boundary exceeded)");
                last_problem_ = true;
            }
            else if (!problem_ && last_problem_)
                last_problem_ = false;
        }
        else
            ROS_ERROR("Error: could not call service arm_health");
            
        last_position_ = new_position;//arm_->getCsvState();
    }
}*/

void ArmControl::executeCommand()
{ 
    switch(command_)
    {
    // Emergency stop-all command gets first evaluation
    case STOP_ALL:
        stopAll();
        command_ = NONE;
        break;
        
    // Arm Control
    case ARM_FORWARD:
        // Toggle
        if (movement_states_[ARM_Z] == 1)
            movement_states_[ARM_Z] = 0;
        else //if (requestMove())
            movement_states_[ARM_Z] = 1;
        break;
    case ARM_BACKWARD:
        // Toggle
        if (movement_states_[ARM_Z] == -1)
            movement_states_[ARM_Z] = 0;
        else //if (requestMove())
            movement_states_[ARM_Z] = -1;
        break;                  
    case ARM_LEFT:
	    // Toggle
        //if (movement_states_[ARM_X] == 1)
        //    movement_states_[ARM_X] = 0;
        //else //if (requestMove())
            movement_states_[ARM_X] = 1;
        break;   
    case ARM_RIGHT:
	    // Toggle
        //if (movement_states_[ARM_X] == -1)
        //    movement_states_[ARM_X] = 0;
        //else //if (requestMove())
            movement_states_[ARM_X] = -1;
        break;         
    case ARM_UP:
	    // Toggle
        //if (movement_states_[ARM_Y] == 1)
        //    movement_states_[ARM_Y] = 0;
        //else //if (requestMove())
            movement_states_[ARM_Y] = 1;
        break;        
    case ARM_DOWN:
	    // Toggle
        //if (movement_states_[ARM_Y] == -1)
        //    movement_states_[ARM_Y] = 0;
        //else //if (requestMove())
            movement_states_[ARM_Y] = -1;
        break; 
    // Folding the arm causes it to get jammed due to all the extra wires
    // present from the camera. Until this is fixed, do not fold/unfold!
    /*    
    case FOLD:
        arm_->fold();
        break;       
    case UNFOLD:
        arm_->unfold();
        break;
    */    
    
    // Claw control        
    case CLAW_YAW_LEFT:
        // Toggle
        if (movement_states_[CLAW_YAW] == 1)
            movement_states_[CLAW_YAW] = 0;
        else //if (requestMove())
            movement_states_[CLAW_YAW] = 1;
        break;         
    case CLAW_YAW_RIGHT:
        // Toggle
        if (movement_states_[CLAW_YAW] == -1)
            movement_states_[CLAW_YAW] = 0;
        else //if (requestMove())
            movement_states_[CLAW_YAW] = -1;
        break;          
    case CLAW_PITCH_UP:
        // Toggle
        if (movement_states_[CLAW_PITCH] == 1)
            movement_states_[CLAW_PITCH] = 0;
        else //if (requestMove())
            movement_states_[CLAW_PITCH] = 1;
        break;        
    case CLAW_PITCH_DOWN:
        // Toggle
        if (movement_states_[CLAW_PITCH] == -1)
            movement_states_[CLAW_PITCH] = 0;
        else //if (requestMove())
            movement_states_[CLAW_PITCH] = -1;
        break;               
    case CLAW_ROLL_LEFT:
        // Toggle
        if (movement_states_[CLAW_ROLL] == -1)
            movement_states_[CLAW_ROLL] = 0;
        else //if (requestMove())
            movement_states_[CLAW_ROLL] = -1;
        break;         
    case CLAW_ROLL_RIGHT:
        // Toggle
        if (movement_states_[CLAW_ROLL] == 1)
            movement_states_[CLAW_ROLL] = 0;
        else //if (requestMove())
            movement_states_[CLAW_ROLL] = 1;
        break;         
    case CLAW_GRIP_OPEN:
        // Toggle
        if (movement_states_[CLAW_GRIP] == 1)
            movement_states_[CLAW_GRIP] = 0;
        else //if (requestMove())
            movement_states_[CLAW_GRIP] = 1;
        break;         
    case CLAW_GRIP_CLOSE:
        // Toggle
        if (movement_states_[CLAW_GRIP] == -1)
            movement_states_[CLAW_GRIP] = 0;
        else //if (requestMove())
            movement_states_[CLAW_GRIP] = -1;
        break; 
    
    // Lift control                
    case LIFT_UP:
        // Toggle
        if (movement_states_[LIFT_UNIT] == -1)
            movement_states_[LIFT_UNIT] = 0;
        else
            movement_states_[LIFT_UNIT] = -1;
        break;                   
    case LIFT_DOWN:
        // Toggle
        if (movement_states_[LIFT_UNIT] == 1)
            movement_states_[LIFT_UNIT] = 0;
        else
            movement_states_[LIFT_UNIT] = 1;
        break;                  
    
    // Other
    case SPEED_DOWN:
        if (movement_states_[SPEED] > 0) movement_states_[SPEED]--;
        printf("Speed[%d]\n", movement_states_[SPEED]);
        break;
    case SPEED_UP:
        if (movement_states_[SPEED] < 4) movement_states_[SPEED]++;
        printf("Speed[%d]\n", movement_states_[SPEED]);
        break;
    case QUERY:
        printStates();
        break;            
    case QUIT:
        stopAll();
        shutdown_ = true;
        break;    
    default:
        break;
    }
    if (!shutdown_ && command_ != NONE && command_ != QUERY)
        move();   
}

/*bool ArmControl::requestMove()
{
    arm::MoveRequest move_request;
    move_request.request.direction = static_cast<int8_t>(command_);
    if (move_request_client_.call(move_request))
    {
	    if (move_request.response.decision == 1)
	        return true;
	    else
	    {
	        ROS_INFO("Cannot move in requested direction (boundary exceeded)");
	    }
	}
	else
	    ROS_ERROR("Error: could not call service move_request");
	return false;
}*/

void ArmControl::printStates()
{
    printf("X[%d] Y[%d] Z[%d]\n", 
           movement_states_[0],
           movement_states_[1],
           movement_states_[2]);
    printf("Yaw[%d] Pitch[%d] Roll[%d] Grip[%d]\n",
            movement_states_[3],
            movement_states_[4],
            movement_states_[5],
            movement_states_[6]);
    printf("Lift[%d] Speed[%d]\n", movement_states_[7], movement_states_[8]);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_control");
    ArmControl arm_control;
    arm_control.init();
    return 0;
}
