// =============================================================================
// Name   : teleop_arm_dish.cpp
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Implements the ROS node "teleop_arm_dish". This node generates commands for
// the "arm_control" node based on activity in the neuron dish.
// =============================================================================

#include "arm/teleop_arm_dish.h"
#include "electrode/cat.h"
#include "arm/movement_states.h"

#define STATES command_.request.states

TeleopArmDish::TeleopArmDish()
{
    cat_sub_ = n_.subscribe("cats", 1000, &TeleopArmDish::callback, this);
    cmd_client_ = n_.serviceClient<arm::command>("commands");
    for (int i = 0; i < 9; i++)
            STATES[i] = 0;
        command_.request.stop_all = false;
        command_.request.query = false;
        command_.request.quit = false;
    STATES[SPEED] = 2;
}

void TeleopArmDish::init()
{
    ROS_INFO("Dish control started...");
    while(ros::ok())
    {
        ros::spinOnce();
        getCommands();
    }
    ROS_INFO("Dish control shutting down...");
}

void TeleopArmDish::getCommands()
{
    if (!queue_.empty())
    {
        electrode::cat cat = queue_.front();
        queue_.pop();
        for (unsigned int i = 1; i < cat.cas.size() && ros::ok(); i++)
        {
            double delta_x = cat.cas[i].x - cat.cas[i-1].x;
            double delta_y = cat.cas[i].y - cat.cas[i-1].y;

            double x_time = 200 * (delta_x >= 0 ? delta_x : -delta_x);
            double y_time = 200 * (delta_y >= 0 ? delta_y : -delta_y);

            printf("Delta: x[%f] y[%f]\n", delta_x, delta_y);
            printf("Time: x[%f] y[%f]\n", x_time, y_time);

            // Determine waiting times
            double first_time = x_time > y_time ? y_time : x_time;
            double second_time = x_time > y_time ? x_time - y_time : y_time -
                                 x_time;

            printf("Time: 1[%f] 2[%f]\n", first_time, second_time);

            // Determine x direction
            if (delta_x > 0)
            {
                printf("Moving left\n");
                STATES[ARM_X] = ARM_LEFT;
            }
            else if (delta_x < 0)
            {
                printf("Moving right\n");
                STATES[ARM_X] = ARM_RIGHT;
            }
            else
            {
                printf("Stopping movement (x)\n");
                STATES[ARM_X] = STOP;
            }

            // Determine y direction
            if (delta_y > 0)
            {
                printf("Moving down\n");
                STATES[ARM_Y] = ARM_DOWN;
            }
            else if (delta_y < 0)
            {
                printf("Moving up\n");
                STATES[ARM_Y] = ARM_UP;
            }
            else
            {
                printf("Stopping movement (y)\n");
                STATES[ARM_Y] = STOP;
            }

            // Move x & y for duration of first_time
            if (cmd_client_.call(command_))
            {
                printf("Command part 1 sent & received\n");
                for (int i = 0; i < 9; i++)
                    STATES[i] = command_.response.states[i];
            }
            else
                ROS_ERROR("Error: could not communicate with command server");
            ros::Duration(first_time).sleep();

            // This part only if x_time and y_time are not equal
            if (second_time > 0)
            {
                // Determine which x axis will stop
                if (x_time > y_time)
                {
                    printf("Stopping movement (y)\n");
                    STATES[ARM_Y] = STOP;
                }
                else
                {
                    printf("Stopping movement (x)\n");
                    STATES[ARM_X] = STOP;
                }

                // Move only x or y for duration of second_time
                if (cmd_client_.call(command_))
                {
                    printf("Command part 2 sent & received\n");
                    for (int i = 0; i < 9; i++)
                        STATES[i] = command_.response.states[i];
                }
                else
                    ROS_ERROR("Error: could not communicate with command server");
                ros::Duration(second_time).sleep();
            }
        }
    }
    else
    {
        printf("Queue empty, stopping all movement\n");
        command_.request.stop_all = true;

        if (cmd_client_.call(command_))
        {
            for (int i = 0; i < 9; i++)
                STATES = command_.response.states;
        }
        else
            ROS_ERROR("Error: could not communicate with command server");

        ros::Duration(1.0).sleep();
    }

    if (command_.request.stop_all)
        command_.request.stop_all = false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_arm_dish");
    TeleopArmDish teleop_arm_dish;
    teleop_arm_dish.init();
    return 0;
}
