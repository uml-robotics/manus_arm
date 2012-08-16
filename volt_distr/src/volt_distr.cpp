// =============================================================================
// Name   : volt_distr.cpp
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Implements the ROS node "volt_distr". This node receives dish states from the
// receiver and records voltage distributions for each channel. It then outputs
// the results to a CSV file and to an image file. The dish states are then
// published again for the burst_creator node.
// =============================================================================

#include "volt_distr/volt_distr.h"

void VoltDistr::init()
{
    dishes_received_ = 0;

    // Get do_forward_dish_states parameter
    if (!n_.getParam("do_forward_dish_states", do_fwd_dish_states_))
    {
        ROS_ERROR("Could not load do_forward_dish_states parameter, default will be used");
        do_fwd_dish_states_ = true;
    }

    // Get buffer_size parameter
    if (!n_.getParam("buffer_size", buffer_size_))
    {
        ROS_ERROR("Could not load buffer_size parameter, default will be used");
        buffer_size_ = 1000;
    }

    // Get loop rate parameter
    int rate;
    if (!n_.getParam("loop_rate", rate))
    {
        ROS_ERROR("Could not load loop_rate parameter, default will be used");
        rate = 200;
    }
    ros::Rate loop_rate(rate);

    // Get volt_distr_log_path parameter
    do_log_volt_distr_ = true;
    if (!n_.getParam("volt_distr_log_path", volt_distr_log_path_))
    {
        ROS_ERROR("Could not load volt_distr_log_path parameter, logging will be disabled");
        do_log_volt_distr_ = false;
    }

    // Get do_truncate_volts parameter
    int do_truncate_volts;
    if (!n_.getParam("do_truncate_volts", do_truncate_volts))
    {
        ROS_ERROR("Could not load do_truncate_volts parameter, default is false");
        do_truncate_volts = 0;
    }
    data_.setDoTruncateVolts(do_truncate_volts);
}

void VoltDistr::callback(const neuro_recv::dish_state::ConstPtr& d)
{

}
