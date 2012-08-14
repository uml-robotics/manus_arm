// =============================================================================
// Name   : volt_distr_creator.cpp
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Voltage distribution creator class for the ROS node "spike_detector". Creates
// a distribution of recorded voltages on each channel.
// =============================================================================

#include "burst_calc/volt_distr_creator.h"
#include "ros/ros.h"

VoltDistrCreator::VoltDistrCreator()
{
    do_truncate_volts_ = false;
}

VoltDistrCreator::~VoltDistrCreator()
{
    if (log_file_.is_open())
        log_file_.close();
}

void VoltDistrCreator::add(const neuro_recv::dish_state& d)
{
    for (int i = 0; i < 60; i++)
    {
        double volt = d.samples[i];

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
}
