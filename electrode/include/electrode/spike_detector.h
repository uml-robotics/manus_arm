// =============================================================================
// Name   : spike_detector.h
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Header file for the ROS node "spike_detector".
// =============================================================================

#ifndef SPIKE_DETECTOR_H_
#define SPIKE_DETECTOR_H_

#include "ros/ros.h"
#include "electrode/buffer_spike_detector.h"
#include "electrode/burst_checker.h"
#include "electrode/burst.h"
#include "electrode/dish_state.h"

class SpikeDetector
{
public:
    SpikeDetector();

private:
    void callback(const electrode::dish_state::ConstPtr& d);
    const electrode::burst cleanBurst(const electrode::burst& b);

    ros::NodeHandle n_;
    ros::Subscriber dish_state_sub_;
    ros::Publisher burst_pub_;
    BufferSpikeDetector buf_;
    BurstChecker bursts_[60];
};

#endif
