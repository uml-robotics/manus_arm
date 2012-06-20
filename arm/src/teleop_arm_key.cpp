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
#include "arm/arm_commands.h"
#include "std_msgs/Int8.h"
#include <signal.h>
#include <termios.h>
#include <stdio.h>

int kfd = 0;
struct termios cooked, raw;

TeleopArm::TeleopArm()
{
    command_pub_ = n_.advertise<std_msgs::Int8>("command", 1);
}

void TeleopArm::init()
{
    ROS_INFO("Keyboard control started...");
    keyLoop();
    ROS_INFO("Keyboard control shutting down...");
}

void TeleopArm::keyLoop()
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
    std_msgs::Int8 command;

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
      
        if (command.data != static_cast<int8_t>(NONE))
        {
            command_pub_.publish(command);
            //ROS_INFO("Command [%d] published", static_cast<int>(command.data));
        }
        
        if (command.data == static_cast<int8_t>(QUIT))
            shutdown = true;
    }
}

int8_t TeleopArm::getCommand(char c)
{
    int command_id;
    switch (c)
    {
    // Arm control
    case KEYCODE_W:
        command_id = ARM_FORWARD;
        break;
    case KEYCODE_A:
        command_id = ARM_LEFT;
        break;
    case KEYCODE_S:
        command_id = ARM_BACKWARD;
        break;
    case KEYCODE_D:
        command_id = ARM_RIGHT;
        break;
    case KEYCODE_Z:
        command_id = ARM_DOWN;
        break;
    case KEYCODE_X:
        command_id = ARM_UP;
        break;
    // Folding the arm causes it to get jammed due to all the extra wires
    // present from the camera. Until this is fixed, do not fold/unfold!
    /*       
    case KEYCODE_F:
        command_id = FOLD;
        break;
         
    case KEYCODE_U:
        command_id = UNFOLD;
        break;
    */
    
    // Claw control     
    case KEYCODE_NUM_4:
        command_id = CLAW_YAW_LEFT;
        break;      
    case KEYCODE_NUM_6:
        command_id = CLAW_YAW_RIGHT;
        break;      
    case KEYCODE_NUM_8:
        command_id = CLAW_PITCH_UP;
        break;      
    case KEYCODE_NUM_2:
        command_id = CLAW_PITCH_DOWN;
        break;      
    case KEYCODE_NUM_7:
        command_id = CLAW_ROLL_LEFT;
        break;      
    case KEYCODE_NUM_9:
        command_id = CLAW_ROLL_RIGHT;
        break;      
    case KEYCODE_NUM_MINUS:
        command_id = CLAW_GRIP_CLOSE;
        break;      
    case KEYCODE_NUM_PLUS:
        command_id = CLAW_GRIP_OPEN;
        break;
    
    // Lift control    
    case KEYCODE_UP:
        command_id = LIFT_UP;
        break;   
    case KEYCODE_DOWN:
        command_id = LIFT_DOWN;
        break;           
       
    // Other
    case KEYCODE_COMMA:
        command_id = SPEED_DOWN;
        break;
    case KEYCODE_PERIOD:
        command_id = SPEED_UP;
        break;
    case KEYCODE_BACKSPACE:
        command_id = ALL_STOP;
        break;
    case KEYCODE_TAB:
        command_id = QUERY;
        break;               
    case KEYCODE_Q:
        command_id = QUIT;
        break;     
    default:
        command_id = NONE;
        break;
    }
    return static_cast<int8_t>(command_id);
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
    TeleopArm teleop_arm;
    teleop_arm.init();
    ros::shutdown(); // Allows other nodes to shutdown afterwards
    return 0;
}
