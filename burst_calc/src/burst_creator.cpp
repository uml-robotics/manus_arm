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
#include <cstdio>

BurstCreator::BurstCreator()
{
    burst_pub_ = n_.advertise<burst_calc::burst>("bursts", 1000);
}

void BurstCreator::init()
{
    ROS_INFO("Burst creator running...");
    burst_pub_ = n_.advertise<burst_calc::burst>("bursts", 1000);

    // Wait for a subscriber to "bursts" before subscribing to "dish_states"
    ROS_INFO("Waiting for subscriber...");
    while (burst_pub_.getNumSubscribers() < 1 && ros::ok());
    ROS_INFO("Subscriber found. Continuing...");

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

    ROS_INFO("Spike detector shutting down...");
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
                printf("*   Burst   : [Sz %4u] [%6.3f - %6.3f] [Ch %2d]\n",
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
            printf("*** Publish : [Sz %4u] [%6.3f - %6.3f]\n",
                   merger_.getBurst().dishes.size(),
                   merger_.getBurst().header.stamp.toSec(),
                   merger_.getBurst().end.toSec());
            merger_.deletePublished();
        }
    }
    else
    {
        buf_.add(d);
        // After every add, check if we now are buffered so we can init the
        // BurstCheckers
        if (buf_.isBuffered())
        {
            for (int i = 0; i < 60; i++)
                bursts_[i].init(i, buf_.getBaseline(i), buf_.getThreshold(i));
        }
    }
}

void BurstCreator::callback(const neuro_recv::dish_state::ConstPtr& d)
{
    queue_.push(*d);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "burst_creator");
    BurstCreator burst_creator;
    burst_creator.init();
    return 0;
}
