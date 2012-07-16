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

#define STDEV_MULT 1.5

BufferSpikeDetector::BufferSpikeDetector()
{
    // Initialize min and max volts to fake values so they will update properly
    for (int i = 0; i < 60; i++)
    {
        min_volts_[i] = 1.0;
        max_volts_[i] = -1.0;
    }
}

void BufferSpikeDetector::calculate()
{
    int size = data_.size();

    // Initialize and calculate each sum
    double sum[60];
    for (int i = 0; i < 60; i++)
        sum[i] = 0.0;
    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < 60; j++)
        {
            sum[j] += data_[i].samples[j];

            if (data_[i].samples[j] > max_volts_[j])
                max_volts_[j] = data_[i].samples[j];
            else if (data_[i].samples[j] < min_volts_[j])
                min_volts_[j] = data_[i].samples[j];
        }
    }

    // Calculate each baseline and initialize each sum of squares
    double sum_squares[60];
    for (int i = 0; i < 60; i++)
    {
        baselines_[i] = sum[i] / size;
        sum_squares[i] = 0.0;
    }

    // Calculate each sum of squares
    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < 60; j++)
            sum_squares[j] += pow(data_[i].samples[j] - baselines_[j], 2);
    }

    // Calculate each standard deviation and threshold
    double stdev[60];
    for (int i = 0; i < 60; i++)
    {
        stdev[i] = sqrt(sum_squares[i] / (size - 1));
        thresholds_[i] = stdev[i] * STDEV_MULT + baselines_[i];
    }
}

burst_calc::ranges BufferSpikeDetector::getRanges()
{
    burst_calc::ranges ranges;
    for (int i = 0; i < 60; i++)
    {
        ranges.baselines[i] = thresholds_[i];
        ranges.min_volts[i] = min_volts_[i];
        ranges.max_volts[i] = max_volts_[i];
    }
    return ranges;
}
