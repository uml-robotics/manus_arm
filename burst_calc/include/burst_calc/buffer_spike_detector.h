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
    bool isBuffered() { return data_.size() >= buffer_size_; }
    burst_calc::ranges getRanges();

private:
    void calculate();

    std::vector<neuro_recv::dish_state> data_;
    boost::array<double, 60> baselines_;
    boost::array<double, 60> thresholds_;
    boost::array<double, 60> min_volts_;
    boost::array<double, 60> max_volts_;
    int buffer_size_;
    double stdev_mult_;
};

#endif
