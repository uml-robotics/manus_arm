/*
 * volt_distr.h
 * Copyright 2013 University of Massachusetts Lowell
 * Author: Jonathan Hasenzahl
 */

#ifndef VOLT_DISTR_H_
#define VOLT_DISTR_H_

#include "ros/ros.h"
#include "volt_distr/volt_distr_creator.h"
#include "volt_distr/volt_distr_viz.h"
#include "neuro_recv/dish_state.h"
#include <string>
#include <fstream>

/*!
 * \brief Node for recording and visualizing voltage distributions.
 *
 * This node receives dish states from a receiver node and then maintains a
 * voltage histogram for each channel. It then will save this data as a CSV
 * file and/or an image.
 *
 * \copyright Copyright 2013 University of Massachusetts Lowell
 * \author Jonathan Hasenzahl
 */
class VoltDistr
{
public:
    VoltDistr() { init(); }
    ~VoltDistr() { delete viz_; }

private:
    void init();
    void getParams();
    void callback(const neuro_recv::dish_state::ConstPtr& d);

    ros::NodeHandle n_;
    ros::Subscriber dish_sub_;

    VoltDistrCreator data_;
    VoltDistrViz* viz_;

    std::string log_path_;
    std::string img_path_;

    bool do_log_;
    bool do_img_;
    bool do_truncate_;
};

#endif
