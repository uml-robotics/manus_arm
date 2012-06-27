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
#include "burst_calc/buffer_spike_detector.h"
#include "burst_calc/burst_checker.h"
#include "burst_calc/burst.h"
#include "neuro_recv/dish_state.h"

class SpikeDetector
{
public:
    SpikeDetector();

private:
    void callback(const neuro_recv::dish_state::ConstPtr& d);
    const burst_calc::burst cleanBurst(const burst_calc::burst& b);

    ros::NodeHandle n_;
    ros::Subscriber dish_state_sub_;
    ros::Publisher burst_pub_;
    BufferSpikeDetector buf_;
    BurstChecker bursts_[60];
};

#endif
