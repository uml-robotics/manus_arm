// =============================================================================
// Name   : burst_checker.h
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Burst checker class for the ROS node "spike_detector". Checks for bursts and
// generates burst sequences. Each burst checker is responsible for a single
// electrode in the neuron dish.
// =============================================================================

#ifndef BURST_CHECKER_H_
#define BURST_CHECKER_H_

#include "electrode/dish_state.h"
#include "electrode/burst.h"

class BurstChecker
{
public:
    BurstChecker()
    {
        frame_count_ = 0;
        spike_count_ = 0;
        is_bursting_ = false;
        is_possible_burst_ = false;
        end_of_burst_ = false;
    }

    void init(const int index, const double baseline, const double threshold)
    {
        index_ = index;
        baseline_ = baseline;
        threshold_ = threshold;
    }

    void reset()
    {
        end_of_burst_ = false;
        burst_.dishes.clear();
    }

    const electrode::burst& getBurst()
    {
        burst_.end.sec = burst_.dishes.back().header.stamp.sec;
        burst_.end.nsec = burst_.dishes.back().header.stamp.nsec;
        return burst_;
    }

    void update(const electrode::dish_state& d);

    bool endOfBurst() { return end_of_burst_; }

private:
    electrode::burst burst_;
    int index_;
    double baseline_;
    double threshold_;
    int frame_count_;
    int spike_count_;
    bool is_bursting_;
    bool is_possible_burst_;
    bool end_of_burst_;
};

#endif
