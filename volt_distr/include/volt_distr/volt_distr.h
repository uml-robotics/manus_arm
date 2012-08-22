// =============================================================================
// Name   : volt_distr.h
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Header file for the ROS node "volt_distr".
// =============================================================================

#ifndef VOLT_DISTR_H_
#define VOLT_DISTR_H_

#include "ros/ros.h"
#include "volt_distr/volt_distr_creator.h"
#include "volt_distr/volt_distr_viz.h"
#include "neuro_recv/dish_state.h"
#include <string>
#include <fstream>

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