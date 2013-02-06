/*
 * burst_checker.h
 * Copyright 2013 University of Massachusetts Lowell
 * Author: Jonathan Hasenzahl
 */

#ifndef BURST_CHECKER_H_
#define BURST_CHECKER_H_

#include "neuro_recv/dish_state.h"
#include "burst_calc/burst.h"

/*!
 * \brief Helper class for the burst_creator node
 *
 * Monitors a single channel for bursting activity. A burst is defined as 3 or
 * more spikes in a period in which the activity does not drop below baseline.
 *
 * \copyright Copyright 2013 University of Massachusetts Lowell
 * \author Jonathan Hasenzahl
 */
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
