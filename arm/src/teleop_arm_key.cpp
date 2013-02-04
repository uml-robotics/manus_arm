/*
 * teleop_arm_key.cpp
 * Copyright 2013 University of Massachusetts Lowell
 */

// =============================================================================
// Name   : teleop_arm_key.cpp
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Implements the ROS node "teleop_arm_key". This node accepts keyboard input to
// pass commands to the "arm_control" node. The code used for getting input from
// the keyboard was originally found in teleop_turtle_key.cpp inside the
// "turtlesim" package on ros.org.
// =============================================================================

#include "arm/teleop_arm_key.h"
#include "arm/movement_definitions.h"
#include <signal.h>
#include <termios.h>
#include <stdio.h>

int kfd = 0;
struct termios cooked, raw;

/*!
 * \brief Default constructor.
 *
 * Starts the command publisher, and the runs the init method.
 */
TeleopArmKey::TeleopArmKey()
{
    cmd_pub_ = n_.advertise<arm::constant_move>("constant_moves", 1);
    cmd_.states.fill(0);
    cmd_.speeds.fill(0);
    cmd_.query = false;
    cmd_.quit = false;
    init();
}

/*!
 * \brief Initializes the node.
 *
 * Waits for a subscriber to the command publisher, and then runs the keyLoop
 * method.
 */
void TeleopArmKey::init()
{
    // Wait for a subscriber to "constant_moves" before continuing
    ROS_INFO("Waiting for subscriber...");
    while (cmd_pub_.getNumSubscribers() < 1 && ros::ok());
    keyLoop();
}

/*!
 * \brief Interprets user keypresses into ARM control commands.
 */
void TeleopArmKey::keyLoop()
{
    // Get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    
    // Setting a new line, then end of file                         
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    printf("\nNow reading from the keyboard.\n");
    printf("Use the keyboard to move the ARM.\n");
    printf("Take care not to exceed its range of motion!\n");
    
    char input;
    bool shutdown = false;
    bool new_command = false;

    // Continue only while shutdown flag has not been enabled and there is still
    // a subscriber to receive messages
    while (!shutdown && cmd_pub_.getNumSubscribers() > 0 && ros::ok())
    {
        // Get the next event from the keyboard  
        if(read(kfd, &input, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }
      
        //printf("Key value: 0x%02X\n", input);
        new_command = getCommand(input);
      
        if (new_command)
        {
            cmd_pub_.publish(cmd_);

            if (cmd_.query)
                cmd_.query = false;
            if (cmd_.quit)
                shutdown = true;

            new_command = false;
        }
    }
}

/*!
 * \brief Determines the command based on the input character.
 * \param c the input character
 * \return true if a new command was registered, false otherwise
 */
bool TeleopArmKey::getCommand(const char c)
{
    bool new_command = true;
    switch (c)
    {
    case KEYCODE_BACKSPACE:
        cmd_.states.fill(0);
        cmd_.speeds.fill(0);
        break;

    // Arm control
    case KEYCODE_W:
        cmd_.states[X] = cmd_.states[X] == ARM_FORWARD ? STOP :
                             ARM_FORWARD;
        break;
    case KEYCODE_A:
        cmd_.states[Y] = cmd_.states[Y] == ARM_LEFT ? STOP : ARM_LEFT;
        break;
    case KEYCODE_S:
        cmd_.states[X] = cmd_.states[X] == ARM_BACKWARD ? STOP :
                             ARM_BACKWARD;
        break;
    case KEYCODE_D:
        cmd_.states[Y] = cmd_.states[Y] == ARM_RIGHT ? STOP : ARM_RIGHT;
        break;
    case KEYCODE_Z:
        cmd_.states[Z] = cmd_.states[Z] == ARM_UP ? STOP : ARM_UP;
        break;
    case KEYCODE_X:
        cmd_.states[Z] = cmd_.states[Z] == ARM_DOWN ? STOP : ARM_DOWN;
        break;
    case KEYCODE_F:
        // Fold: not implemented
        new_command = false;
        break;
    case KEYCODE_U:
        // Unfold: not implemented
        new_command = false;
        break;
    
    // Claw control     
    case KEYCODE_NUM_4:
        cmd_.states[YAW] = cmd_.states[YAW] == CLAW_YAW_LEFT ? STOP :
                                CLAW_YAW_LEFT;
        break;
    case KEYCODE_NUM_6:
        cmd_.states[YAW] = cmd_.states[YAW] == CLAW_YAW_RIGHT ? STOP :
                                CLAW_YAW_RIGHT;
        break;
    case KEYCODE_NUM_8:
        cmd_.states[PITCH] = cmd_.states[PITCH] == CLAW_PITCH_UP ?
                                  STOP : CLAW_PITCH_UP;
        break;
    case KEYCODE_NUM_2:
        cmd_.states[PITCH] = cmd_.states[PITCH] == CLAW_PITCH_DOWN ?
                                  STOP : CLAW_PITCH_DOWN;
        break;
    case KEYCODE_NUM_7:
        cmd_.states[ROLL] = cmd_.states[ROLL] == CLAW_ROLL_LEFT ?
                                 STOP : CLAW_ROLL_LEFT;
        break;
    case KEYCODE_NUM_9:
        cmd_.states[ROLL] = cmd_.states[ROLL] == CLAW_ROLL_RIGHT ?
                                 STOP : CLAW_ROLL_RIGHT;
        break;
    case KEYCODE_NUM_MINUS:
        cmd_.states[GRIP] = cmd_.states[GRIP] == CLAW_GRIP_CLOSE ?
                                 STOP : CLAW_GRIP_CLOSE;
        break;
    case KEYCODE_NUM_PLUS:
        cmd_.states[GRIP] = cmd_.states[GRIP] == CLAW_GRIP_OPEN ?
                                 STOP : CLAW_GRIP_OPEN;
        break;
    
    // Lift control    
    case KEYCODE_UP:
        cmd_.states[LIFT] = cmd_.states[LIFT] == LIFT_UP ? STOP :
                                 LIFT_UP;
        break;
    case KEYCODE_DOWN:
        cmd_.states[LIFT] = cmd_.states[LIFT] == LIFT_DOWN ? STOP :
                                 LIFT_DOWN;
        break;
       
    // Other
    case KEYCODE_COMMA:
    	// Currently affects all speeds at once
        if (cmd_.speeds[0] > 0)
        {
            for (int i = 0; i < SPD_ARR_SZ; i++)
            	cmd_.speeds[i]--;
        }
        else
            new_command = false;

        printf("Speed[%d]\n", cmd_.speeds[0]);
        break;
    case KEYCODE_PERIOD:
    	// Currently affects all speeds at once
        if (cmd_.speeds[0] < 4)
        {
            for (int i = 0; i < SPD_ARR_SZ; i++)
            	cmd_.speeds[i]++;
        }
        else
            new_command = false;

        printf("Speed[%d]\n", cmd_.speeds[0]);
        break;
    case KEYCODE_TAB:
        print();
        cmd_.query = true;
        break;
    case KEYCODE_Q:
        cmd_.states.fill(0);
        cmd_.quit = true;
        break;
    default:
        new_command = false;
        break;
    }

    return new_command;
}

/*!
 * \brief Prints movement states to the screen.
 */
void TeleopArmKey::print()
{
    printf("\nX[%d] Y[%d] Z[%d]\n", cmd_.states[1], cmd_.states[2],
           cmd_.states[0]);
    printf("Yaw[%d] Pitch[%d] Roll[%d] Grip[%d]\n", cmd_.states[3],
           cmd_.states[4], cmd_.states[5], cmd_.states[6]);
    printf("Lift[%d] Speed[%d]\n", cmd_.states[7], cmd_.speeds[0]);
}

/*!
 * \brief
 */
void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    exit(0);
}

/*!
 * \brief Creates an instance of the node
 */
int main(int argc, char** argv)
{
    signal(SIGINT, quit);
    
    ros::init(argc, argv, "teleop_arm_key");
    TeleopArmKey teleop_arm_key;
    ros::shutdown(); // To allow arm_control node to shutdown
    return 0;
}
