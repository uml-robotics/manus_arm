/*
 * volt_distr_creator.h
 * Copyright 2013 University of Massachusetts Lowell
 * Author: Jonathan Hasenzahl
 */

#ifndef VOLT_DISTR_CREATOR_H_
#define VOLT_DISTR_CREATOR_H_

#include "neuro_recv/dish_state.h"
#include <map>
#include <vector>
#include <fstream>

/*!
 * \brief Truncates a double to a max of 4 digits after the decimal
 * \param val the value to be truncated
 * \return the truncated result
 */
double truncate(double val)
{
    if (val > 0)
        return floor(val * 10000) / 10000;
    else if (val < 0)
        return ceil(val * 10000) / 10000;
    else
        return val;
}

/*!
 * \brief Helper class for the volt_distr node
 *
 * Creates a distribution of recorded voltages on each channel.
 *
 * \copyright Copyright 2013 University of Massachusetts Lowell
 * \author Jonathan Hasenzahl
 */
class VoltDistrCreator
{
public:
    VoltDistrCreator();

    void add(const neuro_recv::dish_state& d);
    void toFile(const std::string& file_path);
    boost::array<double, 60> getPercents();

    void setDoTruncateVolts(int i) { do_truncate_volts_ = i; }

private:
    std::map<double, std::vector<int> > volts_;
    std::ofstream log_file_;
    bool do_truncate_volts_;
    boost::array<int, 60> negatives_;
    int total_dishes_;
};

#endif
