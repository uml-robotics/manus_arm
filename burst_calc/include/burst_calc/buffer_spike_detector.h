// =============================================================================
// Name   : buffer_spike_detector.h
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Buffer class for the ROS node "spike_detector". Calculates baseline and
// standard deviation for determining spikes.
// =============================================================================

#ifndef BUFFER_SPIKE_DETECTOR_H_
#define BUFFER_SPIKE_DETECTOR_H_

#include "neuro_recv/dish_state.h"
#include "burst_calc/ranges.h"

class BufferSpikeDetector
{
public:
    BufferSpikeDetector();
    void init(int buffer_size, double stdev_mult);
    void add(const neuro_recv::dish_state& d);
    double getBaseline(int index) { return baselines_[index]; }
    double getThreshold(int index) { return thresholds_[index]; }
    bool isBuffered() { return dishes_received_ >= buffer_size_; }
    burst_calc::ranges getRanges();

private:
    void calculate();

    double sums_[60];
    double sum_squares_[60];
    double baselines_[60];
    double variances_[60];
    double stdevs_[60];
    double thresholds_[60];
    double min_volts_[60];
    double max_volts_[60];
    int buffer_size_;
    int dishes_received_;
    double stdev_mult_;
};

#endif
