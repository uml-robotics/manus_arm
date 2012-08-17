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

    dish_sub_ = n_.subscribe("dish_states_to_volt_distr", 1000, &VoltDistr::callback, this);

    // Get buffer_size parameter
    if (!n_.getParam("buffer_size", buffer_size_))
    {
        ROS_ERROR("Could not load buffer_size parameter, default will be used");
        buffer_size_ = 1000;
    }

    // Get volt_distr_log_path parameter
    do_log_volt_distr_ = true;
    if (!n_.getParam("volt_distr_log_path", volt_distr_log_path_))
    {
        ROS_ERROR("Could not load volt_distr_log_path parameter, logging will be disabled");
        do_log_volt_distr_ = false;
    }

    // Get do_truncate_volts parameter
    if (!n_.getParam("do_truncate_volts", do_truncate_volts_))
    {
        ROS_ERROR("Could not load do_truncate_volts parameter, default is false");
        do_truncate_volts_ = false;
    }
    data_.setDoTruncateVolts(do_truncate_volts_);

    ros::spin();
}

void VoltDistr::callback(const neuro_recv::dish_state::ConstPtr& d)
{
    printf("Callback called\n");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "volt_distr");
    VoltDistr volt_distr;
    return 0;
}
