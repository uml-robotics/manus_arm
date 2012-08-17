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
    // Get parameters
    getParams();

    // Initialize the client and publishers
    time_client_ = n_.serviceClient<time_server::time_srv>("time_service");
    burst_pub_cat_ = n_.advertise<burst_calc::burst>("bursts_to_cat_creator", 1000);
    burst_pub_viz_ = n_.advertise<burst_calc::burst>("bursts_to_dish_viz", 1);
    ranges_pub_cat_ = n_.advertise<burst_calc::ranges>("ranges_to_cat_creator", 1);
    ranges_pub_viz_ = n_.advertise<burst_calc::ranges>("ranges_to_dish_viz", 1);

    // Wait for subscribers before continuing
    ROS_INFO("Waiting for subscribers...");
    while (burst_pub_cat_.getNumSubscribers() < 1 &&
           burst_pub_viz_.getNumSubscribers() < 1 &&
           ranges_pub_cat_.getNumSubscribers() < 1 &&
           ranges_pub_viz_.getNumSubscribers() < 1 && ros::ok());

    // Initialize the subscriber
    dish_state_sub_ = n_.subscribe("dish_states_to_burst_creator", 1000, &BurstCreator::callback,
                                   this);

    // Initialize the BufferSpikeDetector
    buf_.init(buffer_size_, stdev_mult_);

    // Run the node
    run();
}

void BurstCreator::getParams()
{
    // Get buffer_size parameter
    if (!n_.getParam("buffer_size", buffer_size_))
    {
        ROS_ERROR("Could not load buffer_size parameter, default is 1000");
        buffer_size_ = 1000;
    }

    // Get stdev_mult parameter
    if (!n_.getParam("stdev_mult", stdev_mult_))
    {
        ROS_ERROR("Could not load stdev_mult parameter, default is 3.0");
        stdev_mult_ = 3.0;
    }

    // Get burst_window parameter
    if (!n_.getParam("burst_window", burst_window_))
    {
        ROS_ERROR("Could not load burst_window parameter, default is 1000");
        burst_window_ = 1000;
    }
}

void BurstCreator::run()
{
    while (ros::ok())
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
                burst_pub_viz_.publish(merger_.getBurst());
                run_once = false;
            }

            burst_pub_cat_.publish(merger_.getBurst());

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
            ranges_pub_cat_.publish(ranges);
            ranges_pub_viz_.publish(ranges);
        }
    }
}

// Closes out any remaining bursts on the channels after the end of new dish
// states.
void BurstCreator::finish()
{
    ROS_INFO("Finishing burst creator node");

    for (int i = 0; i < 60; i++)
    {
        // Grab any remaining bursts that are hanging around in the checkers
        if (bursts_[i].isBursting())
        {
            merger_.add(bursts_[i].getBurst());
            printf("*   Burst   : [Sz %4d] [%6.3f - %6.3f] [Ch %d]\n",
                   static_cast<int>(bursts_[i].getBurst().dishes.size()),
                   bursts_[i].getBurst().header.stamp.toSec(),
                   bursts_[i].getBurst().end.toSec(), i);
        }

        // Set each time ptr to NULL so the merger won't hang onto the rest of
        // its bursts, waiting for future bursts that will never come
        merger_.updateTime(i, NULL);

        // Update the merger with after giving it all this new info
        merger_.update();

        // Publish any remaining bursts
        while (merger_.canPublish())
        {
            burst_pub_cat_.publish(merger_.getBurst());

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
}

void BurstCreator::callback(const neuro_recv::dish_state::ConstPtr& d)
{
    // Check to see if this is the last dish
    if (d->last_dish)
        finish();
    else
        queue_.push(*d);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "burst_creator");
    BurstCreator burst_creator;
    return 0;
}
