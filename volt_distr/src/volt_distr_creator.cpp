/*
 * volt_distr_creator.cpp
 * Copyright 2013 University of Massachusetts Lowell
 * Author: Jonathan Hasenzahl
 */

#include "volt_distr/volt_distr_creator.h"
#include "ros/ros.h"

VoltDistrCreator::VoltDistrCreator()
{
    do_truncate_volts_ = false;
    negatives_.fill(0);
    total_dishes_ = 0;
}

/*!
 * \brief Adds the values of a dish state to the distribution
 *
 * The class maintains a histogram of voltages, as well as a count of negative
 * voltages, for each channel.
 *
 * \param d the dish state being added
 */
void VoltDistrCreator::add(const neuro_recv::dish_state& d)
{
    total_dishes_++;

    for (int i = 0; i < 60; i++)
    {
        double volt = d.samples[i];

        // Check to see if the voltage is negative
        if (volt < 0.0)
            negatives_[i]++;

        // Truncate the voltage if appropriate
        if (do_truncate_volts_)
            volt = truncate(volt);

        if (volts_.count(volt) > 0)
        {
            // If the voltage has already been logged, increment the count for
            // this channel
            volts_[volt][i] += 1;
        }
        else
        {
            // Otherwise create a new pair for the voltage with a count of 1
            // for this channel
            std::vector<int> channels(60, 0);
            channels[i] = 1;

            std::pair<double, std::vector<int> > new_volt(volt, channels);
            volts_.insert(new_volt);
        }
    }
}

/*!
 * \brief Saves the distrubution data to file in CSV format
 * \param file_path the path of the file to be saved
 */
void VoltDistrCreator::toFile(const std::string& file_path)
{
    log_file_.open(file_path.c_str(), std::ios_base::trunc | std::ios_base::out);

    if (!log_file_.is_open())
    {
        ROS_ERROR("Cannot open %s. CSV logging will be disabled.", file_path.c_str());
        return;
    }

    // Write the header
    log_file_ << "voltage,";
    for (int i = 0; i < 60; i++)
        log_file_ << "channel_" << i << ',';
    log_file_ << '\n';

    // Write the data
    std::map<double, std::vector<int> >::iterator it;
    for (it = volts_.begin(); it != volts_.end(); it++)
    {
        log_file_ << (*it).first << ',';
        for (int i = 0; i < 60; i++)
            log_file_ << (*it).second[i] << ',';
        log_file_ << '\n';
    }

    log_file_.close();
}

/*!
 * \brief Gets the percentage of negative dishes for each channel
 * \return an array of doubles representing the percentage of negative dishes
 *         for each channel
 */
boost::array<double, 60> VoltDistrCreator::getPercents()
{
    boost::array<double, 60> percents;
    for (int i = 0; i < 60; i++)
    {
        percents[i] = static_cast<double>(negatives_[i]) / total_dishes_;
        //printf("%d: %f = %d / %d\n", i, percents[i], negatives_[i], total_dishes_);
    }
    return percents;
}
