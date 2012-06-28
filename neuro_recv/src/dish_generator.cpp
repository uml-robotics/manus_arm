// =============================================================================
// Name   : dish_generator.cpp
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Implements the ROS node "dish_generator". This is a test node used to
// publish pseudo-random "dish_state" messages in lieu of the real thing. New
// version uses Abe's algorithm to generate random numbers.
// =============================================================================

#include "ros/ros.h"
#include "neuro_recv/dish_state.h"
#include <cstdlib>
#include <cstdio>
#include <ctime>
#include <cmath>

#define LIMIT 0.003

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dish_generator");
    ros::NodeHandle n;
    ros::Publisher dish_state_pub = n.advertise<burst_calc::dish_state>
                                               ("dish_states", 1000);
    ros::Rate loop_rate(1000); // 1000 dishes per second
    
    burst_calc::dish_state dish;
    srand(time(NULL));
    
    ROS_INFO("Dish generator running...");
    while (ros::ok())
    {
        for (int i = 0; i < 60; i++)
        {
            double offset = (static_cast<double>(rand() % 600) - 300.0) / 1000000.0;
            if (abs(dish.samples[i] + offset) > LIMIT)
                offset = -offset;
            dish.samples[i] += offset;
        }
        dish_state_pub.publish(dish);
        loop_rate.sleep();
    }
    
    return 0;
}