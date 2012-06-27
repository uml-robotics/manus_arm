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

CatCreator::CatCreator()
{
    cat_pub_ = n_.advertise<burst_calc::cat>("cats", 1000);
    save_to_file_ = false;
}

void CatCreator::init()
{
    ROS_INFO("CAT creator running...");

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
    {
        burst_file_.open("burst_test.csv");
        cat_file_.open("cat_test.csv");
    }

    // Continue only while there is a publisher of "bursts"
    while (burst_sub_.getNumPublishers() > 0 && ros::ok())
        ros::spinOnce();

    ROS_INFO("CAT creator shutting down...");
}

void CatCreator::callback(const burst_calc::burst::ConstPtr& b)
{
    burst_calc::cat cat;
    cat.header.stamp = b->header.stamp;
    cat.end = b->end;
    for (unsigned int i = 0; i < b->dishes.size(); i++)
        cat.cas.push_back(CaCalculator::getCa(b->dishes[i]));
    if (save_to_file_)
        toFile(*b, cat);
    cat_pub_.publish(cat);
}

void CatCreator::toFile(const burst_calc::burst& b, const burst_calc::cat& c)
{
    if (burst_file_.is_open() && cat_file_.is_open())
    {
        burst_file_ << b.header.stamp.toSec() << ',' << b.end.toSec() << ','
                    << ',' << (b.end - b.header.stamp).toSec() << ",\n";
        cat_file_ << c.header.stamp.toSec() << ',' << c.end.toSec() << ','
                  << ',' << (c.end - c.header.stamp).toSec() << ",\n";

        for (int i = 0; i < static_cast<int>(b.dishes.size()); i++)
        {
            burst_file_ << "Dish," << b.dishes[i].header.stamp.sec << ','
                        << b.dishes[i].header.stamp.nsec << ',';
            for (int j = 0; j < 60; j++)
                burst_file_ << b.dishes[i].samples[j] << ',';
            burst_file_ << '\n';

            cat_file_ << c.cas[i].header.stamp.toSec() << ',' << c.cas[i].x
                      << ',' << c.cas[i].y << ",\n";
        }
    }
    else
        printf("Unable to open file!\n");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cat_creator");
    CatCreator cat_creator;
    cat_creator.init();
    return 0;
}
