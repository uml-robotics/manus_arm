// =============================================================================
// Name   : cat_test.cpp
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Implements the ROS node "cat_test". This is a test node used to generate
// "burst" messages for the "cat_creator" node.
// =============================================================================

#include "ros/ros.h"
#include "burst_calc/burst.h"
#include "burst_calc/dish_state.h"

boost::array<double, 60> A = { 0.0001, 0.0002, 0.0001, 0.0003, 0.0001, 0.0002,
                               0.0001, 0.0002, 0.0001, 0.0003, 0.0001, 0.0002, 0.0003, 0.0003,
                               0.0002, 0.0004, 0.0001, 0.0002, 0.0001, 0.0003, 0.0001, 0.0002,
                               0.0001, 0.0002, 0.0001, 0.0003, 0.0001, 0.0002, 0.0003, 0.0003,
                               0.0002, 0.0004, 0.0001, 0.0002, 0.0001, 0.0003, 0.0001, 0.0002,
                               0.0001, 0.0002, 0.0001, 0.0003, 0.0001, 0.0002, 0.0003, 0.0003,
                               0.0002, 0.0004, 0.0001, 0.0002, 0.01, 0.01, 0.0001, 0.0002,
                               0.0001, 0.0002, 0.0001, 0.01, 0.01, 0.0002 };

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cat_test");
    ros::NodeHandle n;
    ros::Publisher p = n.advertise<burst_calc::burst>("bursts", 1000);
    ros::Rate loop_rate(1);

    if (ros::ok())
    {
        burst_calc::dish_state d;
        d.samples = A;

        burst_calc::burst b;
        b.dishes.push_back(d);

        while (p.getNumSubscribers() < 1 && ros::ok());
        p.publish(b);
        loop_rate.sleep();
    }

    return 0;
}

/*
   1  2  3  4  5  6  7  8
1  .  0  1  2  3  4  5  .
2  6  7  8  9  10 11 12 13
3  14 15 16 17 18 19 20 21
4  22 23 24 25 26 27 28 29
5  30 31 32 33 34 35 36 37
6  38 39 40 41 42 43 44 45
7  46 47 48 49 50 51 52 53
8  .  54 55 56 57 58 59 .
 */
