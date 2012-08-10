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
#include <map>

VoltDistrCreator::VoltDistrCreator()
{
    for (int i = 0; i < 60; i++)
        volt_distr_[i].setId(i);

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
        if (do_truncate_volts_)
            volt_distr_[i].add(truncate(d.samples[i]));
        else
            volt_distr_[i].add(d.samples[i]);
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

    std::map<double, int>::iterator it;
    for (int i = 0; i < 60; i++)
    {
        log_file_ << "channel_" << i << ",voltage,";
        for (it = volt_distr_[i].begin(); it != volt_distr_[i].end(); it++)
            log_file_ << (*it).first << ',';

        log_file_ << "\n,frequency,";
        for (it = volt_distr_[i].begin(); it != volt_distr_[i].end(); it++)
            log_file_ << (*it).second << ',';

        log_file_ << "\n\n";
    }
}
