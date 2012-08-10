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
#include "time_server/time_srv.h"
#include <cstdio>

void BurstCreator::init()
{
    time_client_ = n_.serviceClient<time_server::time_srv>("time_service");

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
    ranges_fwd_ = n_.advertise<burst_calc::ranges>("fwd_ranges", 1);

    // Wait for subscribers before continuing
    ROS_INFO("Waiting for subscribers...");
    while (burst_pub_.getNumSubscribers() < 1 &&
           burst_fwd_.getNumSubscribers() < 1 && ros::ok());
    ROS_INFO("Subscribers found. Continuing...");

    dish_state_sub_ = n_.subscribe("dish_states", 1000, &BurstCreator::callback,
                                   this);

    // Wait for a publisher of "dish_states" before continuing
    ROS_INFO("Waiting for publisher...");
    while (dish_state_sub_.getNumPublishers() < 1 && ros::ok());
    ROS_INFO("Publisher found. Continuing...");

    // Main loop
    /*while (ros::ok())
    {
        ros::spinOnce();
        if (!queue_.empty())
            addDish();

    }*/
    ros::spin();
}

void BurstCreator::addDish(const neuro_recv::dish_state& d)
{
    //neuro_recv::dish_state d = queue_.front();
    //queue_.pop();

    if (buf_.isBuffered())
    {
        // For each channel:
        // 1. Update the burst checker
        // 2. Update the time in merger
        // 3. If a burst has ended, add it to the merger and reset the checker
        // 4. Update the merger
        // 5. If the merger has a burst to publish:
        //    1. Publish the burst
        //    2. Delete the burst from the merger
        for (int i = 0; i < 60; i++)
        {
            bursts_[i].update(d);
            merger_.updateTime(i, bursts_[i].getTimePtr());
            if (bursts_[i].endOfBurst())
            {
                printf("*   Burst   : [Sz %4d] [%6.3f - %6.3f] [Ch %d]\n",
                       static_cast<int>(bursts_[i].getBurst().dishes.size()),
                       bursts_[i].getBurst().header.stamp.toSec(),
                       bursts_[i].getBurst().end.toSec(), i);
                merger_.add(bursts_[i].getBurst());
                bursts_[i].reset();
            }
        }

        merger_.update();

        // Flag to seed the time server
        static bool run_once = true;

        while (merger_.canPublish())
        {
            // Seed the time server and start the visualizer
            if (run_once)
            {
                time_server::time_srv seed;
                if (time_client_.call(seed))
                    ROS_INFO("Time server seeded successfully");
                else
                    ROS_ERROR("Time server is not responding");
                burst_fwd_.publish(merger_.getBurst());
                run_once = false;
            }

            burst_pub_.publish(merger_.getBurst());

            printf("*** Publish : [Sz %4d] [%6.3f - %6.3f] [Ch",
                   static_cast<int>(merger_.getBurst().dishes.size()),
                   merger_.getBurst().header.stamp.toSec(),
                   merger_.getBurst().end.toSec());
            for (unsigned int j = 0; j < merger_.getBurst().channels.size(); j++)
                printf(" %d", merger_.getBurst().channels[j]);
            printf("]\n");

            ROS_INFO("Burst %.3f published", merger_.getBurst().header.stamp.toSec());

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
                bursts_[i].init(i, buf_.getBaseline(i), buf_.getThreshold(i),
                                burst_window_);

            burst_calc::ranges ranges = buf_.getRanges();
            ranges_pub_.publish(ranges);
            ranges_fwd_.publish(ranges);
        }
    }
}

void BurstCreator::callback(const neuro_recv::dish_state::ConstPtr& d)
{
    //queue_.push(*d);
    dish_state_fwd_.publish(*d); // Forward the dish state to dish_viz
    addDish(*d);
    //ROS_INFO("Dish state %f received", d->header.stamp.toSec());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "burst_creator");
    BurstCreator burst_creator;
    return 0;
}
