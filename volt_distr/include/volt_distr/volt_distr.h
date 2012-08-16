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
    void callback(const neuro_recv::dish_state::ConstPtr& d);

    ros::NodeHandle n_;
    ros::Subscriber dish_state_sub_;
    ros::Publisher dish_state_pub_;
    VoltDistrCreator data_;
    int dishes_received_;
    std::string volt_distr_log_path_;
    bool do_fwd_dish_states_;
    bool do_log_volt_distr_;
    int buffer_size_;
};

#endif
