// =============================================================================
// Name   : volt_distr.h
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Voltage distribution class for the ROS node "spike_detector". Contains a
// distribution of recorded voltages on a single channel.
// =============================================================================

#ifndef VOLT_DISTR_H_
#define VOLT_DISTR_H_

#include <map>
#include <cmath>

class VoltDistr
{
public:
    VoltDistr() {}

    void add(double volt)
    {
        if (data_.count(volt) == 0)
        {
            // This voltage doesn't exist in the map, so insert a new pair with
            // a count of 1
            std::pair<double, int> new_volt(volt, 1);
            data_.insert(new_volt);
        }
        else
        {
            // This voltage already exists in the map, so increment the count
            data_[volt] += 1;
        }
    }

    std::map<double, int>::iterator begin() { return data_.begin(); }
    std::map<double, int>::iterator end() { return data_.end(); }

    void setId(int id) { id_ = id; }
    int getId() { return id_; }

private:
    int id_;
    std::map<double, int> data_;
};

#endif
