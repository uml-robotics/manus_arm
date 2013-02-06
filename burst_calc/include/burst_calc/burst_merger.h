/*
 * burst_merger.h
 * Copyright 2013 University of Massachusetts Lowell
 * Author: Jonathan Hasenzahl
 */

#ifndef BURST_MERGER_H_
#define BURST_MERGER_H_

#include "ros/ros.h"
#include "neuro_recv/dish_state.h"
#include "burst_calc/burst.h"
#include <vector>

/*!
 * \brief Helper class for the burst_creator node
 *
 * Checks for overlapping bursts from each channel. If there are overlapping
 * bursts, they will be merged so duplicates are eliminated.
 *
 * \copyright Copyright 2013 University of Massachusetts Lowell
 * \author Jonathan Hasenzahl
 */
class BurstMerger
{
public:
    BurstMerger();
    ~BurstMerger();
    void update();
    void updateTime(int index, const ros::Time* t);

    void add(const burst_calc::burst& b) { list_.push_back(b); }
    bool canPublish() { return !final_.empty(); }
    const burst_calc::burst& getBurst() { return final_[0]; };
    void deletePublished() { final_.erase(final_.begin()); }

private:
    burst_calc::burst merge(const burst_calc::burst& b1,
                            const burst_calc::burst& b2);

    std::vector<burst_calc::burst> final_;
    std::vector<burst_calc::burst> list_;
    boost::array<ros::Time*, 60> times_;
};

#endif
