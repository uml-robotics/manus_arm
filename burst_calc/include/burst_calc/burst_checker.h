// =============================================================================
// Name   : burst_checker.h
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Burst checker class for the ROS node "spike_detector". Checks for bursts and
// generates burst sequences. Each burst checker is responsible for a single
// burst_calc in the neuron dish.
// =============================================================================

#ifndef BURST_CHECKER_H_
#define BURST_CHECKER_H_

#include "neuro_recv/dish_state.h"
#include "burst_calc/burst.h"

class BurstChecker
{
public:
    BurstChecker();
    void init(int index, double baseline, double threshold, int burst_window);
    void update(const neuro_recv::dish_state& d);
    void reset();
    const ros::Time* getTimePtr();

    const burst_calc::burst& getBurst() { return burst_; }
    bool isBursting() { return is_bursting_; }
    bool endOfBurst() { return end_of_burst_; }

private:
    burst_calc::burst burst_;
    int index_;
    double baseline_;
    double threshold_;
    int frame_count_;
    int spike_count_;
    bool is_bursting_;
    bool is_possible_burst_;
    bool end_of_burst_;
    int burst_window_;
};

#endif
