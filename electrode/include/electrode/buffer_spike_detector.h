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

#include "electrode/dish_state.h"

#define BUFFER_SIZE 1000

class BufferSpikeDetector
{
public:
    BufferSpikeDetector() {}
    void add(const electrode::dish_state& d)
    {
        data_.push_back(d);
        if (isBuffered())
            calculate();
    }
    double getBaseline(int index) { return baseline_[index]; }
    double getThreshold(int index) { return threshold_[index]; }
    bool isBuffered() { return data_.size() >= BUFFER_SIZE; }

private:
    void calculate();

    std::vector<electrode::dish_state> data_;
    boost::array<double, 60> baseline_;
    boost::array<double, 60> threshold_;
};

#endif
