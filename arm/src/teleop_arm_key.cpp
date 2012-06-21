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
#include "arm/key_codes.h"
#include "arm/movement_states.h"
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define STATES command_.request.states

int kfd = 0;
struct termios cooked, raw;

TeleopArmKey::TeleopArmKey()
{
    cmd_client_ = n_.serviceClient<arm::command>("commands");
    for (int i = 0; i < 9; i++)
        STATES[i] = 0;
    command_.request.stop_all = false;
    command_.request.query = false;
    command_.request.quit = false;
}

void TeleopArmKey::init()
{
    ROS_INFO("Keyboard control started...");
    keyLoop();
    ROS_INFO("Keyboard control shutting down...");
}

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
    printf("Take care not to exceed its range of motion!\n\n");
    
    char input;
    bool shutdown = false;
    bool new_command = false;

    while (ros::ok() && !shutdown)
    {
        // Get the next event from the keyboard  
        if(read(kfd, &input, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }
      
        //ROS_INFO("value: 0x%02X\n", input);
        new_command = getCommand(input);
      
        if (new_command)
        {
            if (cmd_client_.call(command_))
                STATES = command_.response.states;
            else
                ROS_ERROR("Error: could not communicate with command server");

            if (command_.request.stop_all)
                command_.request.stop_all = false;
            else if (command_.request.query)
                command_.request.query = false;
            else if (command_.request.quit)
                shutdown = true;

            new_command = false;
        }
    }
}

bool TeleopArmKey::getCommand(const char c)
{
    switch (c)
    {
    // Emergency stop-all command gets first evaluation
    case KEYCODE_BACKSPACE:
        command_.request.stop_all = true;
        return true;

    // Arm control
    case KEYCODE_W:
        STATES[ARM_Z] = STATES[ARM_Z] == ARM_FORWARD ? STOP : ARM_FORWARD;
        return true;
    case KEYCODE_A:
        STATES[ARM_X] = STATES[ARM_X] == ARM_LEFT ? STOP : ARM_LEFT;
        return true;
    case KEYCODE_S:
        STATES[ARM_Z] = STATES[ARM_Z] == ARM_BACKWARD ? STOP : ARM_BACKWARD;
        return true;
    case KEYCODE_D:
        STATES[ARM_X] = STATES[ARM_X] == ARM_RIGHT ? STOP : ARM_RIGHT;
        return true;
    case KEYCODE_Z:
        STATES[ARM_Y] = STATES[ARM_Y] == ARM_UP ? STOP : ARM_UP;
        return true;
    case KEYCODE_X:
        STATES[ARM_Y] = STATES[ARM_Y] == ARM_DOWN ? STOP : ARM_DOWN;
        return true;
    case KEYCODE_F:
        // Fold: not implemented
        return false;
    case KEYCODE_U:
        // Unfold: not implemented
        return false;
    
    // Claw control     
    case KEYCODE_NUM_4:
        STATES[CLAW_YAW] = STATES[CLAW_YAW] == CLAW_YAW_LEFT ? STOP :
                           CLAW_YAW_LEFT;
        return true;
    case KEYCODE_NUM_6:
        STATES[CLAW_YAW] = STATES[CLAW_YAW] == CLAW_YAW_RIGHT ? STOP :
                           CLAW_YAW_RIGHT;
        return true;
    case KEYCODE_NUM_8:
        STATES[CLAW_PITCH] = STATES[CLAW_PITCH] == CLAW_PITCH_UP ? STOP :
                             CLAW_PITCH_UP;
        return true;
    case KEYCODE_NUM_2:
        STATES[CLAW_PITCH] = STATES[CLAW_PITCH] == CLAW_PITCH_DOWN ? STOP :
                             CLAW_PITCH_DOWN;
        return true;
    case KEYCODE_NUM_7:
        STATES[CLAW_ROLL] = STATES[CLAW_ROLL] == CLAW_ROLL_LEFT ? STOP :
                            CLAW_ROLL_LEFT;
        return true;
    case KEYCODE_NUM_9:
        STATES[CLAW_ROLL] = STATES[CLAW_ROLL] == CLAW_ROLL_RIGHT ? STOP :
                            CLAW_ROLL_RIGHT;
        return true;
    case KEYCODE_NUM_MINUS:
        STATES[CLAW_GRIP] = STATES[CLAW_GRIP] == CLAW_GRIP_CLOSE ? STOP :
                            CLAW_GRIP_CLOSE;
        return true;
    case KEYCODE_NUM_PLUS:
        STATES[CLAW_GRIP] = STATES[CLAW_GRIP] == CLAW_GRIP_OPEN ? STOP :
                            CLAW_GRIP_OPEN;
        return true;
    
    // Lift control    
    case KEYCODE_UP:
        STATES[LIFT_UNIT] = STATES[LIFT_UNIT] == LIFT_UP ? STOP : LIFT_UP;
        return true;
    case KEYCODE_DOWN:
        STATES[LIFT_UNIT] = STATES[LIFT_UNIT] == LIFT_DOWN ? STOP : LIFT_DOWN;
        return true;
       
    // Other
    case KEYCODE_COMMA:
        if (STATES[SPEED] > 0)
        {
            STATES[SPEED]--;
            printf("Speed[%d]\n", STATES[SPEED]);
            return true;
        }
        else
            return false;
    case KEYCODE_PERIOD:
        if (STATES[SPEED] < 4)
        {
            STATES[SPEED]++;
            printf("Speed[%d]\n", STATES[SPEED]);
            return true;
        }
        else
            return false;
    case KEYCODE_TAB:
        command_.request.query = true;
        return true;
    case KEYCODE_Q:
        command_.request.quit = true;
        return true;
    default:
        return false;
    }
}

void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    exit(0);
}

int main(int argc, char** argv)
{
    signal(SIGINT, quit);
    
    ros::init(argc, argv, "teleop_arm_key");
    TeleopArmKey teleop_arm_key;
    teleop_arm_key.init();
    return 0;
}
