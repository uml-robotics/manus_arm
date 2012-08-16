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
#include <string>

class BurstCreator
{
public:
    BurstCreator() { init(); }

private:
    void init();
    void getParams();
    void run();
    void addDish();
    void finish();
    void callback(const neuro_recv::dish_state::ConstPtr& d);

    ros::NodeHandle n_;
    ros::Subscriber dish_state_sub_;
    ros::Publisher burst_pub_cat_;
    ros::Publisher burst_pub_viz_;
    ros::Publisher ranges_pub_cat_;
    ros::Publisher ranges_pub_viz_;
    ros::ServiceClient time_client_;

    std::queue<neuro_recv::dish_state> queue_;

    BufferSpikeDetector buf_;
    BurstChecker bursts_[60];
    BurstMerger merger_;

    int buffer_size_;
    int stdev_mult_;
    int burst_window_;
};

#endif
