/*
 * burst_creator.h
 * Copyright 2013 University of Massachusetts Lowell
 * Author: Jonathan Hasenzahl
 */

#ifndef BURST_CREATOR_H_
#define BURST_CREATOR_H_

#include "ros/ros.h"
#include "burst_calc/buffer_spike_detector.h"
#include "burst_calc/burst_checker.h"
#include "burst_calc/burst_merger.h"
#include "burst_calc/burst.h"
#include "neuro_recv/dish_state.h"
#include <queue>
#include <string>

/*!
 * \brief Node for creating bursts
 *
 * This node receives dish states from a receiver node and then detects and
 * creates burst sequences, which are forwarded to the CAT creator node. The
 * operation of the node is autonomous; after the object has been created, the
 * node will run with no further operation.
 *
 * \copyright Copyright 2013 University of Massachusetts Lowell
 * \author Jonathan Hasenzahl
 */
class BurstCreator
{
public:
    BurstCreator() { init(); }

private:
    void init();
    void getParams();
    void run();
    void addDish();
    void finish();
    void callback(const neuro_recv::dish_state::ConstPtr& d);

    ros::NodeHandle n_;
    ros::Subscriber dish_state_sub_;
    ros::Publisher burst_pub_cat_;
    ros::Publisher burst_pub_viz_;
    ros::Publisher ranges_pub_cat_;
    ros::Publisher ranges_pub_viz_;
    ros::ServiceClient time_client_;

    std::queue<neuro_recv::dish_state> queue_;

    BufferSpikeDetector buf_;
    BurstChecker bursts_[60];
    BurstMerger merger_;

    int buffer_size_;
    double stdev_mult_;
    int burst_window_;
};

#endif
