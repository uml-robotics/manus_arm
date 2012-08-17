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
#include "neuro_recv/dish_state.h"
#include <string>

class VoltDistr
{
public:
    VoltDistr() { init(); }


private:
    void init();
    void getParams();
    void callback(const neuro_recv::dish_state::ConstPtr& d);

    ros::NodeHandle n_;
    ros::Subscriber dish_sub_;
    VoltDistrCreator data_;
    std::string volt_distr_log_path_;
    bool do_log_volt_distr_;
    bool do_truncate_volts_;
};

#endif
