// =============================================================================
// Name   : cat_creator.cpp
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Implements the ROS node "cat_creator". This node calculates and publishes
// CATs (center of activity trajectories). A CAT is a sequence of CAs  (centers
// of activity).
// =============================================================================

#include "burst_calc/cat_creator.h"
#include <cstdio>

CatCreator::~CatCreator()
{
    if (cat_file_.is_open())
        cat_file_.close();
}

void CatCreator::init()
{
    is_init_ = false;
    save_to_file_ = true;

    getParams();

    // Initialize publisher
    cat_pub_ = n_.advertise<burst_calc::cat>("cats", 1000);

    // Wait for subscriber
    ROS_INFO("Waiting for subscriber...");
    while (cat_pub_.getNumSubscribers() < 1 && ros::ok());

    // Initialize subscribers
    burst_sub_ = n_.subscribe("bursts_to_cat_creator", 1000, &CatCreator::callback, this);
    ranges_sub_ = n_.subscribe("ranges_to_cat_creator", 1, &CatCreator::rangesCallback,
                               this);

    if (save_to_file_)
        initFile(file_name_.c_str());

    ros::spin();
}

void CatCreator::getParams()
{
    // Get cat log path parameter
    if (!n_.getParam("cat_log_path", file_name_))
    {
        ROS_ERROR("Could not load burst_log_path parameter, logging will be disabled");
        save_to_file_ = false;
    }
}

void CatCreator::updateOffsets(const neuro_recv::dish_state& d)
{
    for (int i = 0; i < 60; i++)
    {
        if (d.samples[i] < offsets_[i])
        {
            offsets_[i] = d.samples[i];
            printf("Offset correction: channel %d set to %f\n", i, d.samples[i]);
        }
    }
}

void CatCreator::callback(const burst_calc::burst::ConstPtr& b)
{
    if (is_init_)
    {
        // All the work is done in the callback. Each dish state has a CA calculated
        // and added to the vector in the CAT message. The CAT is then logged and
        // published.
        burst_calc::cat cat;
        cat.header.stamp = b->header.stamp;
        cat.end = b->end;
        cat.channels = b->channels;
        for (unsigned int i = 0; i < b->dishes.size(); i++)
        {
            updateOffsets(b->dishes[i]);
            if (caExists(b->dishes[i]))
                cat.cas.push_back(getCa(b->dishes[i]));
        }
        if (save_to_file_)
            toFile(*b, cat);
        cat_pub_.publish(cat);

        ROS_INFO("CAT of size %d created from burst of size %d",
                 static_cast<int>(cat.cas.size()), static_cast<int>(b->dishes.size()));
    }
    else
        ROS_ERROR("Minimum voltages not initialized, skipping CAT creation");
}

void CatCreator::rangesCallback(const burst_calc::ranges::ConstPtr& r)
{
    printf("rangesCallback called");
    for (int i = 0; i < 60; i++)
    {
        offsets_[i] = r->min_volts[i];
        thresholds_[i] = r->thresholds[i];
    }

    if (save_to_file_)
    {
        cat_file_ << "threshold_volts,,,,";
        for (int i = 0; i < 60; i++)
            cat_file_ << r->thresholds[i] << ',';
        cat_file_ << "\n\n";
    }

    is_init_ = true;
}

bool CatCreator::caExists(const neuro_recv::dish_state& d)
{
    // Only dish states with at least 1 spike will have a CA calculated
    for (int i = 0; i < 60; i ++)
    {
        if (d.samples[i] > thresholds_[i])
            return true;
    }

    // No spikes in this dish state
    return false;
}

const burst_calc::ca CatCreator::getCa(const neuro_recv::dish_state& d)
{
    // Center of activity = summation(position*activity) / total activity
    double x_sum = 0.0;
    double y_sum = 0.0;
    double activity = 0.0;

    for (int i = 0; i < 60; i++)
    {
        // Only spikes are used for CA calculation
        if (d.samples[i] > thresholds_[i])
        {
            double this_activity = d.samples[i] - offsets_[i];
            if (this_activity < 0.0)
            {
                ROS_ERROR("Activity is lower than recorded minimum, CA will not be accurate");
                ROS_ERROR("%d: %f = %f - %f\n", i, this_activity, d.samples[i],
                          offsets_[i]);
                this_activity = -this_activity;
            }

            x_sum += this_activity * X_COORD_[i];
            y_sum += this_activity * Y_COORD_[i];
            activity += this_activity;
        }
    }

    burst_calc::ca ca;
    ca.header.stamp = d.header.stamp;
    if (activity > 0.0)
    {
        ca.x = x_sum / activity;
        ca.y = y_sum / activity;
    }
    else
        ROS_ERROR("No activity recorded for this CA");

    //printf("ca.x = %f / %f = %f\n", x_sum, activity, ca.x);
    //printf("ca.y = %f / %f = %f\n\n", y_sum, activity, ca.y);

    return ca;
}

void CatCreator::initFile(const char* cat_file)
{
    cat_file_.open(cat_file, std::ios_base::trunc | std::ios_base::out);
    if (!cat_file_.is_open())
    {
        ROS_ERROR("Cannot open %s. CSV logging will be disabled.", cat_file);
        save_to_file_ = false;
        return;
    }

    cat_file_ << "index,seconds,ca_x,ca_y,";
    for (int i = 0; i < 60; i++)
    {
        cat_file_ << "channel_" << i << ',';
    }
    cat_file_ << '\n';
}

void CatCreator::toFile(const burst_calc::burst& b, const burst_calc::cat& c)
{
    cat_file_ << "begin_time," << c.header.stamp.toSec() << ",\n";
    cat_file_ << "end_time," << c.end.toSec() << ",\n";
    cat_file_ << "duration," << (c.end - c.header.stamp).toSec() << ",\n";
    cat_file_ << "bursting_channels,\n";
    for (unsigned int i = 0; i < c.channels.size(); i++)
        cat_file_ << static_cast<int>(c.channels[i]) << ",\n";

    for (int i = 0; i < static_cast<int>(c.cas.size()); i++)
    {
        cat_file_ << i << ',' << c.cas[i].header.stamp.toSec() << ','
                  << c.cas[i].x << ',' << c.cas[i].y << ',';
        for (int j = 0; j < 60; j++)
            cat_file_ << b.dishes[i].samples[j] << ',';
        cat_file_ << '\n';
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cat_creator");
    CatCreator cat_creator;
    return 0;
}
