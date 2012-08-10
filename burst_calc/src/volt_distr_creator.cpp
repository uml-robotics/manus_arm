// =============================================================================
// Name   : volt_distr_creator.cpp
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Voltage distribution creator class for the ROS node "spike_detector". Creates
// a distribution of recorded voltages on each channel.
// =============================================================================

#include "burst_calc/volt_distr_creator.h"

VoltDistrCreator::VoltDistrCreator()
{
    for (int i = 0; i < 60; i++)
        volt_distr_[i].setId(i);
}

void VoltDistrCreator::add(const neuro_recv::dish_state& d)
{
    for (int i = 0; i < 60; i++)
        volt_distr_[i].add(d.samples[i]);
}
