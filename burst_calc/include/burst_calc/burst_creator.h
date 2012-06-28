// =============================================================================
// Name   : burst_creator.h
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Header file for the ROS node "burst_creator".
// =============================================================================

#ifndef BURST_CREATOR_H_
#define BURST_CREATOR_H_

#include "ros/ros.h"
#include "burst_calc/buffer_spike_detector.h"
#include "burst_calc/burst_checker.h"
#include "burst_calc/burst_merger.h"
#include "burst_calc/burst.h"
#include "neuro_recv/dish_state.h"
#include <queue>

class BurstCreator
{
public:
    BurstCreator() { init(); }
    void init();

private:
    void addDish();
    void callback(const neuro_recv::dish_state::ConstPtr& d);

    ros::NodeHandle n_;
    ros::Subscriber dish_state_sub_;
    ros::Publisher burst_pub_;
    //ros::Publisher stream_pub_;
    std::queue<neuro_recv::dish_state> queue_;
    BufferSpikeDetector buf_;
    BurstChecker bursts_[60];
    BurstMerger merger_;
};

#endif
