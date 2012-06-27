// =============================================================================
// Name   : burst_merger.h
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Burst merger class for the ROS node "spike_detector". Ensures bursts are
// unique and merges them if necessary.
// =============================================================================

#ifndef BURST_MERGER_H_
#define BURST_MERGER_H_

#include "ros/ros.h"
#include "neuro_recv/dish_state.h"
#include "burst_calc/burst.h"
#include <vector>

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
