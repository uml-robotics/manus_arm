// =============================================================================
// Name   : burst_creator.cpp
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Implements the ROS node "burst_creator". This node determines when incoming
// dish state activity is spiking, and then creates and publishes
// non-overlapping bursts.
// =============================================================================

#include "burst_calc/burst_creator.h"
#include "burst_calc/ranges.h"
#include <cstdio>

void BurstCreator::init()
{
    // Get buffer_size parameter
    int buffer_size;
    if (!n_.getParam("buffer_size", buffer_size))
    {
        ROS_ERROR("Could not load buffer_size parameter, default will be used");
        buffer_size = 1000;
    }

    // Get stdev_mult parameter
    double stdev_mult;
    if (!n_.getParam("stdev_mult", stdev_mult))
    {
        ROS_ERROR("Could not load stdev_mult parameter, default will be used");
        stdev_mult = 3.0;
    }

    // Get burst_window parameter
    if (!n_.getParam("burst_window", burst_window_))
    {
        ROS_ERROR("Could not load burst_window parameter, default will be used");
        burst_window_ = 1000;
    }

    // Init the BufferSpikeDetector
    buf_.init(buffer_size, stdev_mult);

    // Init the subscribers and publishers
    burst_pub_ = n_.advertise<burst_calc::burst>("bursts", 1000);
    burst_fwd_ = n_.advertise<burst_calc::burst>("fwd_bursts", 1);
    dish_state_fwd_ = n_.advertise<neuro_recv::dish_state>("fwd_dish_states",
                                                           1000);
    ranges_pub_ = n_.advertise<burst_calc::ranges>("ranges", 1);

    // Wait for subscribers before continuing
    ROS_INFO("Waiting for subscribers...");
    while (burst_pub_.getNumSubscribers() < 1 &&
           burst_fwd_.getNumSubscribers() < 1 &&
           dish_state_fwd_.getNumSubscribers() < 1 && ros::ok());
    ROS_INFO("Subscribers found. Continuing...");

    dish_state_sub_ = n_.subscribe("dish_states", 1000, &BurstCreator::callback,
                                   this);

    // Wait for a publisher of "dish_states" before continuing
    ROS_INFO("Waiting for publisher...");
    while (dish_state_sub_.getNumPublishers() < 1 && ros::ok());
    ROS_INFO("Publisher found. Continuing...");

    // Continue only while there is a publisher of "dish_states"
    while (dish_state_sub_.getNumPublishers() > 0 && ros::ok())
    {
        ros::spinOnce();
        if (!queue_.empty())
            addDish();
    }
}

void BurstCreator::addDish()
{
    neuro_recv::dish_state d = queue_.front();
    queue_.pop();

    if (buf_.isBuffered())
    {
        for (int i = 0; i < 60; i++)
        {
            bursts_[i].update(d);
            merger_.updateTime(i, bursts_[i].getTimePtr());
            if (bursts_[i].endOfBurst())
            {
                printf("*   Burst   : [Sz %4u] [%6.3f - %6.3f] [Ch %d]\n",
                       bursts_[i].getBurst().dishes.size(),
                       bursts_[i].getBurst().header.stamp.toSec(),
                       bursts_[i].getBurst().end.toSec(), i);
                merger_.add(bursts_[i].getBurst());
                bursts_[i].reset();
            }
        }

        merger_.update();

        while (merger_.canPublish())
        {
            burst_pub_.publish(merger_.getBurst());

            bool run_once = true;
            if (run_once)
            {
                burst_fwd_.publish(merger_.getBurst());
                run_once = false;
            }

            printf("*** Publish : [Sz %4u] [%6.3f - %6.3f] [Ch",
                   merger_.getBurst().dishes.size(),
                   merger_.getBurst().header.stamp.toSec(),
                   merger_.getBurst().end.toSec());
            for (unsigned int j = 0; j < merger_.getBurst().channels.size(); j++)
                printf(" %d", merger_.getBurst().channels[j]);
            printf("]\n");

            merger_.deletePublished();
        }
    }
    else
    {
        buf_.add(d);
        // After every add, check if we now are buffered so we can init the
        // BurstCheckers. The volt ranges are also published to dish_viz.
        if (buf_.isBuffered())
        {
            for (int i = 0; i < 60; i++)
            {
                bursts_[i].init(i, buf_.getBaseline(i), buf_.getThreshold(i),
                                burst_window_);
                printf("%d: Baseline[%.3f] Threshold[%.3f]\n", i,
                       buf_.getBaseline(i), buf_.getThreshold(i));
            }

            ranges_pub_.publish(buf_.getRanges());
        }
    }
}

void BurstCreator::callback(const neuro_recv::dish_state::ConstPtr& d)
{
    queue_.push(*d);
    dish_state_fwd_.publish(*d);
    //ROS_INFO("Dish state %f received", d->header.stamp.toSec());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "burst_creator");
    BurstCreator burst_creator;
    return 0;
}
