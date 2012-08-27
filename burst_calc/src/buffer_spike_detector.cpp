// =============================================================================
// Name   : buffer_spike_detector.cpp
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Buffer class for the ROS node "spike_detector". Calculates baseline and
// standard deviation for determining spikes.
// =============================================================================

#include "burst_calc/buffer_spike_detector.h"
#include <cstdio>
#include <cmath>

BufferSpikeDetector::BufferSpikeDetector()
{
    dishes_received_ = 0;

    for (int i = 0; i < 60; i++)
    {
        sums_[i] = 0.0;
        sum_squares_[i] = 0.0;
        min_volts_[i] = 1.0;
        max_volts_[i] = -1.0;
    }
}

void BufferSpikeDetector::init(int buffer_size, double stdev_mult)
{
    buffer_size_ = buffer_size;
    stdev_mult_ = stdev_mult;
}

void BufferSpikeDetector::add(const neuro_recv::dish_state& d)
{
    dishes_received_++;

    // Update min/max and add to sum
    for (int i = 0; i < 60; i++)
    {
        if (d.samples[i] > max_volts_[i])
        {
            if (i == 0)
                printf("%.5f > Max (%.5f)\n", d.samples[i], max_volts_[i]);
            max_volts_[i] = d.samples[i];
        }

        if (d.samples[i] < min_volts_[i])
        {
            if (i == 0)
                printf("%.5f < Min (%.5f)\n", d.samples[i], min_volts_[i]);
            min_volts_[i] = d.samples[i];
        }

        sums_[i] += d.samples[i];
        sum_squares_[i] += d.samples[i] * d.samples[i];
    }

    // Calculate the baselines and thresholds if there are enough dish states
    if (isBuffered())
        calculate();
}

void BufferSpikeDetector::calculate()
{
    for (int i = 0; i < 60; i++)
    {
        baselines_[i] = sums_[i] / dishes_received_;
        variances_[i] = (sum_squares_[i] - ((sums_[i] * sums_[i]) / dishes_received_)) / (dishes_received_ - 1);
        stdevs_[i] = sqrt(variances_[i]);
        thresholds_[i] = stdevs_[i] * stdev_mult_ + baselines_[i];
        printf("Min: %+.5f Max: %+.5f Base: %+.5f StDev: %+.5f Thresh: %+.5f\n",
               min_volts_[i], max_volts_[i], baselines_[i], stdevs_[i],
               thresholds_[i]);
    }
}

burst_calc::ranges BufferSpikeDetector::getRanges()
{
    burst_calc::ranges ranges;
    for (int i = 0; i < 60; i++)
    {
        ranges.baselines[i] = baselines_[i];
        ranges.thresholds[i] = thresholds_[i];
        ranges.min_volts[i] = min_volts_[i];
        ranges.max_volts[i] = max_volts_[i];
    }
    return ranges;
}
