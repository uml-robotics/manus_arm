// =============================================================================
// Name   : visualizer.h
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Header file for the ROS node "visualizer".
// =============================================================================

#ifndef VISUALIZER_H_
#define VISUALIZER_H_

#include "ros/ros.h"
//#include "electrode/spike_state.h"

struct coord
{
    double x, y;
    bool exists;
};

class Visualizer
{
public:
    Visualizer();
    void init();

private:
    coord centerOfActivity();
    void display();
    void sleep(int seconds);
    //void callback(const electrode::spike_state::ConstPtr& dish);

    ros::NodeHandle n_;
    ros::Subscriber spike_state_sub_;
    bool grid_[8][8];
};

#endif
