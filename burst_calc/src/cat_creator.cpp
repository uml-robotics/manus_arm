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
#include "burst_calc/ca_calculator.h"
#include "burst_calc/cat.h"
#include <cstring>
#include <cstdio>

CatCreator::~CatCreator()
{
    if (burst_file_.is_open())
        burst_file_.close();
    if (cat_file_.is_open())
        cat_file_.close();
}

void CatCreator::init(bool save_to_file, char* burst_file, char* cat_file)
{
    cat_pub_ = n_.advertise<burst_calc::cat>("cats", 1000);
    save_to_file_ = save_to_file;

    // Wait for a subscriber to "cats" before subscribing to "bursts"
    ROS_INFO("Waiting for subscriber...");
    while (cat_pub_.getNumSubscribers() < 1 && ros::ok());
    ROS_INFO("Subscriber found. Continuing...");

    burst_sub_ = n_.subscribe("bursts", 1000, &CatCreator::callback, this);

    // Wait for a publisher of "bursts" before continuing
    ROS_INFO("Waiting for publisher...");
    while (burst_sub_.getNumPublishers() < 1 && ros::ok());
    ROS_INFO("Publisher found. Continuing...");

    if (save_to_file_)
        initFile(burst_file, cat_file);

    // Continue only while there is a publisher of "bursts"
    while (burst_sub_.getNumPublishers() > 0 && ros::ok())
        ros::spinOnce();
}

void CatCreator::callback(const burst_calc::burst::ConstPtr& b)
{
    burst_calc::cat cat;
    cat.header.stamp = b->header.stamp;
    cat.end = b->end;
    cat.channels = b->channels;
    for (unsigned int i = 0; i < b->dishes.size(); i++)
        cat.cas.push_back(CaCalculator::getCa(b->dishes[i]));
    if (save_to_file_)
        toFile(*b, cat);
    cat_pub_.publish(cat);
}

void CatCreator::initFile(char* burst_file, char* cat_file)
{
    burst_file_.open(burst_file);
    if (burst_file_.is_open())
    {
        burst_file_.close();
        remove(burst_file);
        burst_file_.open(burst_file);
    }
    else
    {
        ROS_WARN("Cannot open %s. CSV logging will be disabled.", burst_file);
        save_to_file_ = false;
        return;
    }

    cat_file_.open(cat_file);
    if (cat_file_.is_open())
    {
        cat_file_.close();
        remove(cat_file);
        cat_file_.open(cat_file);
    }
    else
    {
        ROS_WARN("Cannot open %s. CSV logging will be disabled.", cat_file);
        save_to_file_ = false;
        return;
    }

    burst_file_ << "index,sec,nsec,";
    for (int i = 0; i < 60; i++)
    {
        burst_file_ << "channel_" << i << ',';
    }
    burst_file_ << '\n';

    cat_file_ << "index,sec,nsec,x_coord,y_coord,\n";
}

void CatCreator::toFile(const burst_calc::burst& b, const burst_calc::cat& c)
{

    burst_file_ << "burst_begin," << b.header.stamp.sec << ','
                << b.header.stamp.nsec << ",\n";
    burst_file_ << "burst_end," << b.end.sec << ',' << b.end.nsec << ",\n";
    burst_file_ << "burst_duration," << (b.end - b.header.stamp).sec << ','
                << (b.end - b.header.stamp).nsec << ",\n";
    burst_file_ << "bursting_channels,";
    for (unsigned int i = 0; i < b.channels.size(); i++)
        burst_file_ << static_cast<int>(b.channels[i]) << ',';
    burst_file_ << '\n';

    cat_file_ << "cat_begin," << c.header.stamp.sec << ','
                << c.header.stamp.nsec << ",\n";
    cat_file_ << "cat_end," << c.end.sec << ',' << c.end.nsec << ",\n";
    cat_file_ << "cat_duration," << (c.end - c.header.stamp).sec << ','
                << (c.end - c.header.stamp).nsec << ",\n";
    cat_file_ << "bursting_channels,";
    for (unsigned int i = 0; i < c.channels.size(); i++)
        cat_file_ << static_cast<int>(c.channels[i]) << ',';
    cat_file_ << '\n';

    for (int i = 0; i < static_cast<int>(b.dishes.size()); i++)
    {
        burst_file_ << i << ',' << b.dishes[i].header.stamp.sec << ','
                    << b.dishes[i].header.stamp.nsec << ',';
        for (int j = 0; j < 60; j++)
            burst_file_ << b.dishes[i].samples[j] << ',';
        burst_file_ << '\n';

        cat_file_ << i << ',' << c.cas[i].header.stamp.sec << ','
                  << c.cas[i].header.stamp.nsec << ',' << c.cas[i].x << ','
                  << c.cas[i].y << ",\n";
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cat_creator");
    CatCreator cat_creator;

    if (argc > 1)
    {
        if (strcmp(argv[1], "--file") == 0 || strcmp(argv[1], "-f") == 0)
        {
            if (argc == 4)
                cat_creator.init(true, argv[2], argv[3]);
            else
                ROS_ERROR("You must specify exactly two file names.");
        }
        else
            ROS_ERROR("Option %s not recognized. Use --file or -f.", argv[1]);
    }
    else
        cat_creator.init(false, NULL, NULL);

    return 0;
}
