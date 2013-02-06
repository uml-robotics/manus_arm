/*
 * burst_checker.cpp
 * Copyright 2013 University of Massachusetts Lowell
 * Author: Jonathan Hasenzahl
 */

#include "burst_calc/burst_checker.h"
#include <cstdio>

/*!
 * \brief Sets values to their initial states
 */
BurstChecker::BurstChecker()
{
    frame_count_ = 0;
    spike_count_ = 0;
    is_bursting_ = false;
    is_possible_burst_ = false;
    end_of_burst_ = false;
}

/*!
 * \brief Gives the object values needed for later calculations
 * \param index the index of the channel, from 0 to 59
 * \param baseline the baseline voltage of the channel
 * \param threshold the spike threshold of the channel
 * \param burst_window the size (in dish states) of the burst window
 */
void BurstChecker::init(int index, double baseline, double threshold,
                        int burst_window)
{
    burst_.channels.push_back(index);
    index_ = index;
    baseline_ = baseline;
    threshold_ = threshold;
    burst_window_ = burst_window;
}

/*!
 * \brief Updates the burst checker with a new dish state
 *
 * This method contains the logic for determining if a burst is occurring on
 * this channel.
 *
 * \param d the dish state being added
 */
void BurstChecker::update(const neuro_recv::dish_state& d)
{
    if (d.samples[index_] <= baseline_)
    {
        // Newest sample is not above baseline:
        // If a there is a possible burst, end it.
        if (is_possible_burst_)
        {
            if (is_bursting_)
            {
                end_of_burst_ = true;
                is_bursting_ = false;
            }
            is_possible_burst_ = false;
            frame_count_ = 0;
            spike_count_ = 0;
            // Clear the burst sequence if there was no actual burst
            if (!end_of_burst_)
                burst_.dishes.clear();
        }
    }
    else if (d.samples[index_] <= threshold_)
    {
        // Newest sample is between baseline and threshold:
        // If there is a possible burst, increment frame count. If frame count
        // doesn't exceed the window, add the dish state to the sequence.
        // Otherwise, end the possible burst.
        if (is_possible_burst_)
        {
            if (++frame_count_ <= burst_window_)
            {
                burst_.dishes.push_back(d);
                burst_.end = d.header.stamp;
            }
            else
            {
                if (is_bursting_)
                {
                    end_of_burst_ = true;
                    is_bursting_ = false;
                }
                is_possible_burst_ = false;
                frame_count_ = 0;
                spike_count_ = 0;
                // Clear the burst sequence if there was no actual burst
                if (!end_of_burst_)
                    burst_.dishes.clear();
            }
        }
    }
    else
    {
        // Newest sample is a spike:
        // Add the spike state to the sequence and increment frame count and
        // spike count. If there was no possible burst before, there is now. If
        // this is the 3rd spike, a burst is occurring.
        frame_count_++;
        spike_count_++;
        if (!is_possible_burst_)
        {
            is_possible_burst_ = true;
            burst_.header.stamp = d.header.stamp;
        }
        if (spike_count_ == 3)
            is_bursting_ = true;
        burst_.dishes.push_back(d);
        burst_.end = d.header.stamp;
    }

    /*printf("State: %f Frames: %d Spikes: %d Possible: %s Burst: %s End: %s\n",
           d.samples[index_],
           frame_count_,
           spike_count_,
           is_possible_burst_ ? "Yes" : "No",
           is_bursting_ ? "Yes" : "No",
           end_of_burst_ ? "Yes" : "No");*/
}

/*!
 * \brief Resets the burst checker
 *
 * This should be called after the end of a burst has been detected (isBursting)
 * and the burst has been retrieved (getBurst).
 */
void BurstChecker::reset()
{
    end_of_burst_ = false;
    burst_.dishes.clear();
}

/*!
 * \brief Returns a pointer to the start time of the burst
 *
 * \return if there is a possible burst, returns a pointer to the start time of
 * the; returns a null pointer otherwise
 */

const ros::Time* BurstChecker::getTimePtr()
{
    if (is_possible_burst_)
        return &burst_.header.stamp;
    else
        return NULL;
}
