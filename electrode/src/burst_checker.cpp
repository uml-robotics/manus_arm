// =============================================================================
// Name   : burst_checker.cpp
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Burst checker class for the ROS node "spike_detector". Checks for bursts and
// generates burst sequences. Each burst checker is responsible for a single
// electrode in the neuron dish.
// =============================================================================

#include "electrode/burst_checker.h"
#include <cstdio>

#define BURST_WINDOW 1000 // Approx. 1000 dishes per second

void BurstChecker::update(const electrode::dish_state& d)
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
            if (++frame_count_ <= BURST_WINDOW)
                burst_.dishes.push_back(d);
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
            burst_.header.stamp.sec = d.header.stamp.sec;
            burst_.header.stamp.nsec = d.header.stamp.nsec;
        }
        if (spike_count_ == 3)
            is_bursting_ = true;
        burst_.dishes.push_back(d);
    }

    /*printf("State: %f Frames: %d Spikes: %d Possible: %s Burst: %s End: %s\n",
           d.samples[index_],
           frame_count_,
           spike_count_,
           is_possible_burst_ ? "Yes" : "No",
           is_bursting_ ? "Yes" : "No",
           end_of_burst_ ? "Yes" : "No");*/
}
