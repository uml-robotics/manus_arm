// =============================================================================
// Name   : arm_monitor.cpp
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Implements the ROS node "arm_monitor". This node continually receives
// positional data from the "arm_control" node and alerts it if the ARM is
// exceeding a movement boundary.
// =============================================================================

#include "arm/arm_monitor.h"
#include <stdio.h>

ArmMonitor::ArmMonitor()
{
    arm_health_service_ = n_.advertiseService("arm_health",
                                              &ArmMonitor::armHealthCallback, 
                                              this);
    move_request_service_ = n_.advertiseService("move_request", 
                                                &ArmMonitor::moveRequestCallback, 
                                                this);
    problem_code_ = 0;                                          
    run_once_ = true;                                           
    shutdown_ = false;
}

void ArmMonitor::init()
{
    ROS_INFO("ARM monitor started...");
    while (ros::ok() && !shutdown_)
    {
        ros::spinOnce();
    }
    print();
    ROS_INFO("ARM monitor shutting down...");
}

bool ArmMonitor::armHealthCallback(arm::ArmHealth::Request &request, 
                                   arm::ArmHealth::Response &response)
{
    if (request.state == "shutdown")
    {
        shutdown_ = true;
        response.decision = 0;
    }
    else
    {
        parse(request.state);
        updateMinMax();
        updateProblemCode();
        response.decision = (problem_code_ != 0 ? 1 : 0);
    }
    return true;
}

bool ArmMonitor::moveRequestCallback(arm::MoveRequest::Request &request,
                                     arm::MoveRequest::Response &response)
{
    int direction = static_cast<int>(request.direction);
    int8_t decision = 1; // Assume it's a go
    
    if (problem_code_ != 0)
    {
        switch (problem_code_)
        {
        /*
        case 50:
            if (direction == ARM_BACKWARD) decision = 0; 
            break;
        case 51:
            if (direction == ARM_RIGHT) decision = 0;
            break;
        case 52:
            if (direction == ARM_DOWN) decision = 0;
            break;    
        case 53:
            if (direction == CLAW_YAW_RIGHT) decision = 0;
            break;
        case 54:
            if (direction == CLAW_ROLL_LEFT) decision = 0;
            break;
        case 55:
            if (direction == CLAW_PITCH_DOWN) decision = 0;
            break;
        */
        case 56:
            if (direction == CLAW_GRIP_CLOSE) decision = 0;
            break;
        /*                
        case 100:
            if (direction == ARM_FORWARD) decision = 1;
            break;
        case 101:
            if (direction == ARM_LEFT) decision = 0;
            break;
        case 102:
            if (direction == ARM_UP) decision = 0;
            break;    
        case 103:
            if (direction == CLAW_YAW_LEFT) decision = 0;
            break;
        case 104:
            if (direction == CLAW_ROLL_RIGHT) decision = 0;
            break;
        case 105:
            if (direction == CLAW_PITCH_UP) decision = 0;
            break;
        */
        case 106:
            if (direction == CLAW_GRIP_OPEN) decision = 0;
            break;       
        default:
            break;
        }
    }
    
    response.decision = decision;       
    return true;
}

// Updates problem_code_ if the ARM has exceeded one of its movement boundaries.
// If the max boundary of state_[i] is exceeded, i+100 is stored in 
// problem_code_. For the min boundary, i+50 is stored:
//      Z     : 100 for MAX, 50 for MIN
//      X     : 101 for MAX, 51 for MIN
//      Y     : 102 for MAX, 52 for MIN
//      YAW   : 103 for MAX, 53 for MIN
//      ROLL  : 104 for MAX, 54 for MIN
//      PITCH : 105 for MAX, 55 for MIN
//      GRIP  : 106 for MAX, 56 for MIN
void ArmMonitor::updateProblemCode()
{   
    /*
    bool stop = false; 
    for (int i = 0; i < SIZE && !stop; i++)
    {
        if (state_[i] > MAX_STATE[i])
        { 
            problem_code_ = i + 100;
            stop = true;
        }
        else if (state_[i] < MIN_STATE[i])
        {
            problem_code_ = i + 50;
            stop = true;
        }
        else problem_code_ = 0; 
    }
    */
    double delta_grip = deltaGrip();
    if (delta_grip < 0 && state_[GRIP] < MIN_STATE[GRIP] && state_[GRIP] > 10000.0)
        problem_code_ = 56;
    else if (delta_grip > 0 && state_[GRIP] > MAX_STATE[GRIP] && state_[GRIP] < 8000.0)
        problem_code_ = 106;
    else
        problem_code_ = 0;
}

void ArmMonitor::parse(std::string s)
{
    printf("%s\n", s.c_str());
    for (int i = 0; i < SIZE - 1; i++)
    {
       int n = s.find(',');
       state_[i] = atof(s.substr(0, n).c_str());
       s.erase(0, n + 1); 
    }
    state_[SIZE - 1] = atof(s.c_str());
    
    state_[GRIP] += 10000.0;
}

double ArmMonitor::deltaGrip()
{
    double delta_grip = state_[GRIP] - last_grip_state_;
    /*printf("Grip Position: %9.3f Delta: %9.3f\n",
           state_[GRIP], delta_grip);*/
    last_grip_state_ = state_[GRIP];
    
    return delta_grip;
}

void ArmMonitor::updateMinMax()
{
    // Populate the min and max arrays with initial values
    if (run_once_)
    {
        for (int i = 0; i < SIZE; i++)
        {
            origin_[i] = state_[i];
            min_state_[i] = state_[i];
            max_state_[i] = state_[i];
        }
        last_grip_state_ = state_[GRIP]; 
        run_once_ = false;
    }
    else
    {
        for (int i = 0; i < SIZE; i++)
        {
            if (state_[i] > max_state_[i])
                max_state_[i] = state_[i];
            else if (state_[i] < min_state_[i])
                min_state_[i] = state_[i];
        }
    }
}

void ArmMonitor::print()
{
    printf("          Origin       Min       Max\n");
    printf("Z%15.3f%10.3f%10.3f\n", origin_[Z], min_state_[Z], max_state_[Z]);
    printf("X%15.3f%10.3f%10.3f\n", origin_[X], min_state_[X], max_state_[X]);
    printf("Y%15.3f%10.3f%10.3f\n", origin_[Y], min_state_[Y], max_state_[Y]);
    printf("Yaw%13.3f%10.3f%10.3f\n", origin_[YAW], min_state_[YAW], max_state_[YAW]);
    printf("Pitch%11.3f%10.3f%10.3f\n", origin_[PITCH], min_state_[PITCH], max_state_[PITCH]);
    printf("Roll%12.3f%10.3f%10.3f\n", origin_[ROLL], min_state_[ROLL], max_state_[ROLL]);
    printf("Grip%12.3f%10.3f%10.3f\n", origin_[GRIP], min_state_[GRIP], max_state_[GRIP]);
}                       

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_monitor");
    ArmMonitor arm_monitor;
    arm_monitor.init();
    return 0;
}
