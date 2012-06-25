// =============================================================================
// Name   : cat_printer.cpp
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Implements the ROS node "cat_printer". This is a test node used to
// inspect the "cat" messages published by the "cat_creator" node.
// =============================================================================
// !!! BROKEN AS OF 6/15/12 DUE TO CHANGES TO CAT CREATOR NODE !!!

#include "ros/ros.h"
#include "electrode/cat.h"
#include <cstdio>

void callback(const electrode::cat::ConstPtr& c)
{
    printf("Trajectory size: %d\n", static_cast<int>(c->cas.size()));
    /*for (unsigned int i = 0; i < t->cat.size(); i++)
    {
        printf("%d[x(%.3f)y(%.3f)] ", i, t->cat[i].x, t->cat[i].y);
    }
    printf("\n");*/
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cat_printer");
    ros::NodeHandle n;
    ros::Subscriber cat_sub = n.subscribe("cats", 1000, callback);
    ROS_INFO("Trajectory printer running...");
    ros::spin();
    return 0;
}
