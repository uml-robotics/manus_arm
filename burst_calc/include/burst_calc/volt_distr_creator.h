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

#include "volt_distr.h"
#include "neuro_recv/dish_state.h"
#include <fstream>

// Truncates a double to a max of 4 digits after the decimal
double truncate(double val)
{
    if (val > 0)
        return floor(val * 10000) / 10000;
    else if (val < 0)
        return ceil(val * 10000) / 10000;
    else
        return val;
}

class VoltDistrCreator
{
public:
    VoltDistrCreator();
    ~VoltDistrCreator();

    void add(const neuro_recv::dish_state& d);
    void toFile(const std::string& file_path);

    void setDoTruncateVolts(int i) { do_truncate_volts_ = i; }

private:
    VoltDistr volt_distr_[60];
    std::ofstream log_file_;
    bool do_truncate_volts_;
};

#endif
