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
    do_log_ = true;
    do_img_ = true;

    // Get parameters
    getParams();

    // Initialize subscriber
    dish_sub_ = n_.subscribe("dish_states_to_volt_distr", 1000, &VoltDistr::callback, this);

    // Initialize log and image objects
    data_.setDoTruncateVolts(do_truncate_);
    viz_ = new VoltDistrViz(img_path_);

    ros::spin();
}

void VoltDistr::getParams()
{
    // Get volt_distr_log_path parameter
    if (!n_.getParam("volt_distr_log_path", log_path_))
    {
        ROS_WARN("Could not load volt_distr_log_path parameter, logging will be disabled");
        do_log_ = false;
    }

    // Get volt_distr_img_path parameter
    if (!n_.getParam("volt_distr_img_path", img_path_))
    {
        ROS_WARN("Could not load volt_distr_img_path parameter, imaging will be disabled");
        do_img_ = false;
    }

    // Get do_truncate_volts parameter
    if (!n_.getParam("do_truncate_volts", do_truncate_))
    {
        ROS_WARN("Could not load do_truncate_volts parameter, default is false");
        do_truncate_ = false;
    }
}

void VoltDistr::callback(const neuro_recv::dish_state::ConstPtr& d)
{
    if (d->last_dish)
    {
        if (do_log_)
        {
            ROS_INFO("Writing voltage distributions to CSV");
            data_.toFile(log_path_);
        }
        if (do_img_)
        {
            ROS_INFO("Creating voltage distribution image");
            viz_->draw(data_.getPercents());
        }
    }
    else
        data_.add(*d);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "volt_distr");
    VoltDistr volt_distr;
    return 0;
}
