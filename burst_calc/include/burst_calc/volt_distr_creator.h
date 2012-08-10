// =============================================================================
// Name   : volt_distr_creator.h
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Voltage distribution creator class for the ROS node "spike_detector". Creates
// a distribution of recorded voltages on each channel.
// =============================================================================

#ifndef VOLT_DISTR_CREATOR_H_
#define VOLT_DISTR_CREATOR_H_

#include "neuro_recv/dish_state.h"
#include "volt_distr.h"

class VoltDistrCreator
{
public:
    VoltDistrCreator();
    void add(const neuro_recv::dish_state& d);
    void toFile();

private:
    VoltDistr volt_distr_[60];
};

#endif
