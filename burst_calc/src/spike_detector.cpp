// =============================================================================
// Name   : spike_detector.cpp
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Implements the ROS node "spike_detector".
// =============================================================================

#include "burst_calc/spike_detector.h"
#include <cstdio>

SpikeDetector::SpikeDetector()
{
    ROS_INFO("Spike detector running...");
    burst_pub_ = n_.advertise<burst_calc::burst>("bursts", 1000);

    // Wait for a subscriber to "bursts" before subscribing to "dish_states"
    ROS_INFO("Waiting for subscriber...");
    while (burst_pub_.getNumSubscribers() < 1 && ros::ok());
    ROS_INFO("Subscriber found. Continuing...");

    dish_state_sub_ = n_.subscribe("dish_states",
                                   1000,
                                   &SpikeDetector::callback,
                                   this);

    // Wait for a publisher of "dish_states" before continuing
    ROS_INFO("Waiting for publisher...");
    while (dish_state_sub_.getNumPublishers() < 1 && ros::ok());
    ROS_INFO("Publisher found. Continuing...");

    // Continue only while there is a publisher of "dish_states"
    while (dish_state_sub_.getNumPublishers() > 0 && ros::ok())
        ros::spinOnce();

    ROS_INFO("Spike detector shutting down...");
}

void SpikeDetector::callback(const neuro_recv::dish_state::ConstPtr& d)
{
    if (buf_.isBuffered())
    {
        for (int i = 0; i < 60; i++)
        {
            bursts_[i].update(*d);
            if (bursts_[i].endOfBurst())
            {
                //while (burst_pub_.getNumSubscribers() < 1 && ros::ok());
                burst_pub_.publish(bursts_[i].getBurst());
                printf("Burst from burst_calc %d of size %d published\n", i,
                       static_cast<int>(bursts_[i].getBurst().dishes.size()));
                bursts_[i].reset();
                //ros::shutdown(); // for testing
            }
        }
    }
    else
    {
        buf_.add(*d);
        // After every add, check if now we are buffered in order to init the
        // BurstCheckers
        if (buf_.isBuffered())
        {
            for (int i = 0; i < 60; i++)
                bursts_[i].init(i, buf_.getBaseline(i), buf_.getThreshold(i));
        }
    }
}

// "Cleans" a burst sequence so that readings below baseline are discarded. This
// is done to increase the difference in Center of Activity calculations. Not
// sure yet if it is actually useful.
// TODO: Implement time stamp in header if I plan on using this again.
const burst_calc::burst SpikeDetector::cleanBurst(const burst_calc::burst& b)
{
    burst_calc::burst clean_burst;
    for (unsigned int i = 0; i < b.dishes.size(); i++)
    {
        neuro_recv::dish_state d;
        for (int j = 0; j < 60; j++)
        {
            if (b.dishes[i].samples[j] > buf_.getBaseline(j))
                d.samples[j] = b.dishes[i].samples[j];
            else
                d.samples[j] = 9.99;
        }
        clean_burst.dishes.push_back(d);
    }
    return clean_burst;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "spike_detector");
    SpikeDetector spike_detector;
    return 0;
}
