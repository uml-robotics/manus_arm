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
#include "arm/command.h"
#include "arm/key_codes.h"
#include "arm/arm_commands.h"
#include <signal.h>
#include <termios.h>
#include <stdio.h>

int kfd = 0;
struct termios cooked, raw;

TeleopArmKey::TeleopArmKey()
{
    command_pub_ = n_.advertise<arm::command>("commands", 1);
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
    arm::command command;

    while (ros::ok() && !shutdown)
    {
        // Get the next event from the keyboard  
        if(read(kfd, &input, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }
      
        //ROS_INFO("value: 0x%02X\n", input);
        command.data = getCommand(input);
      
        if (command.data != NONE)
        {
            command_pub_.publish(command);
            //ROS_INFO("Command [%d] published", static_cast<int>(command.data));
        }
        
        if (command.data == QUIT)
            shutdown = true;
    }
}

uint8_t TeleopArmKey::getCommand(const char c)
{
    switch (c)
    {
    // Emergency stop-all command gets first evaluation
    case KEYCODE_BACKSPACE:
        return STOP_ALL;

    // Arm control
    case KEYCODE_W:
        return ARM_FORWARD;
    case KEYCODE_A:
        return ARM_LEFT;
    case KEYCODE_S:
        return ARM_BACKWARD;
    case KEYCODE_D:
        return ARM_RIGHT;
    case KEYCODE_Z:
        return ARM_UP;
    case KEYCODE_X:
        return ARM_DOWN;
    /* Folding the arm causes it to get jammed due to all the extra wires
     * present from the camera. Until this is fixed, do not fold/unfold!
    case KEYCODE_F:
        return FOLD;
    case KEYCODE_U:
        return UNFOLD;
    */
    
    // Claw control     
    case KEYCODE_NUM_4:
        return CLAW_YAW_LEFT;
    case KEYCODE_NUM_6:
        return CLAW_YAW_RIGHT;
    case KEYCODE_NUM_8:
        return CLAW_PITCH_UP;
    case KEYCODE_NUM_2:
        return CLAW_PITCH_DOWN;
    case KEYCODE_NUM_7:
        return CLAW_ROLL_LEFT;
    case KEYCODE_NUM_9:
        return CLAW_ROLL_RIGHT;
    case KEYCODE_NUM_MINUS:
        return CLAW_GRIP_CLOSE;
    case KEYCODE_NUM_PLUS:
        return CLAW_GRIP_OPEN;
    
    // Lift control    
    case KEYCODE_UP:
        return LIFT_UP;
    case KEYCODE_DOWN:
        return LIFT_DOWN;
       
    // Other
    case KEYCODE_COMMA:
        return SPEED_DOWN;
    case KEYCODE_PERIOD:
        return SPEED_UP;
    case KEYCODE_TAB:
        return QUERY;
    case KEYCODE_Q:
        return QUIT;
    default:
        return NONE;
    }
}

void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

int main(int argc, char** argv)
{
    signal(SIGINT, quit);
    
    ros::init(argc, argv, "teleop_arm_key");
    TeleopArmKey teleop_arm_key;
    ros::shutdown(); // Allows other nodes to shutdown afterwards
    return 0;
}
