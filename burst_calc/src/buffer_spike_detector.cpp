/*
 * buffer_spike_detector.cpp
 * Copyright 2013 University of Massachusetts Lowell
 * Author: Jonathan Hasenzahl
 */

#include "burst_calc/buffer_spike_detector.h"
#include <cstdio>
#include <cmath>

/*!
 * \brief Sets values to their initial states
 */
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

/*!
 * \brief Gives the object values needed for later calculations
 * \param buffer_size the number of dish states used to calculate baselines and
 *                    thresholds
 * \param stdev_mult the standard deviation multiplier used when calculating
 *                   thresholds
 */
void BufferSpikeDetector::init(int buffer_size, double stdev_mult)
{
    buffer_size_ = buffer_size;
    stdev_mult_ = stdev_mult;
}

/*!
 * \brief Adds a dish state to the buffer
 * \param d the dish state to be added
 */
void BufferSpikeDetector::add(const neuro_recv::dish_state& d)
{
    dishes_received_++;

    // Update min/max and add to sum
    for (int i = 0; i < 60; i++)
    {
        if (d.samples[i] > max_volts_[i])
            max_volts_[i] = d.samples[i];
        else if (d.samples[i] < min_volts_[i])
            min_volts_[i] = d.samples[i];

        sums_[i] += d.samples[i];
        sum_squares_[i] += d.samples[i] * d.samples[i];
    }

    // Calculate the baselines and thresholds if there are enough dish states
    if (isBuffered())
        calculate();
}

/*!
 * \brief Calculates values for each channel
 *
 * The values calculated are: baseline, variance, standard deviation, threshold.
 * This method is not called until the buffer is full.
 */
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

/*!
 * \brief Returns a range object with values for each channel
 *
 * The values stored are: baseline, threshold, min voltage, max voltage.
 *
 * \return a range object with values for each channel
 */
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
