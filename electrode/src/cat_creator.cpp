// =============================================================================
// Name   : cat_creator.cpp
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Implements the ROS node "cat_creator". This node calculates and publishes
// CATs (center of activity trajectories). A CAT is a sequence of CAs  (centers
// of activity).
// =============================================================================

#include "electrode/cat_creator.h"
#include "electrode/ca_calculator.h"
#include "electrode/cat.h"

CatCreator::CatCreator()
{
    burst_sub_ = n_.subscribe("bursts", 1000, &CatCreator::callback, this);
    cat_pub_ = n_.advertise<electrode::cat>("cats", 1000);
    ROS_INFO("CAT creator running...");
    //burst_file_.open("burst_test.csv");
    //cat_file_.open("cat_test.csv");
    ros::spin();
}

void CatCreator::callback(const electrode::burst::ConstPtr& b)
{
    electrode::cat cat;
    cat.header.stamp.sec = b->header.stamp.sec;
    cat.header.stamp.nsec = b->header.stamp.nsec;
    cat.end.sec = b->end.sec;
    cat.end.nsec = b->end.nsec;
    for (unsigned int i = 0; i < b->dishes.size(); i++)
        cat.cas.push_back(CaCalculator::getCa(b->dishes[i]));
    //toFile(*b, cat);
    cat_pub_.publish(cat);
}

void CatCreator::toFile(const electrode::burst& b, const electrode::cat& c)
{
    if (burst_file_.is_open() && cat_file_.is_open())
    {
        burst_file_ << b.header.stamp.sec << ',' << b.header.stamp.nsec << ','
                    << b.end.sec << ',' << b.end.nsec << ','
                    << (b.end - b.header.stamp).sec << ','
                    << (b.end - b.header.stamp).nsec << ",\n";
        cat_file_ << c.header.stamp.sec << ',' << c.header.stamp.nsec << ','
                  << c.end.sec << ',' << c.end.nsec << ','
                  << (c.end - c.header.stamp).sec << ','
                  << (c.end - c.header.stamp).nsec << ",\n";

        for (int i = 0; i < static_cast<int>(b.dishes.size()); i++)
        {
            burst_file_ << "Dish," << b.dishes[i].header.stamp.sec << ','
                        << b.dishes[i].header.stamp.nsec << ',';
            for (int j = 0; j < 60; j++)
                burst_file_ << b.dishes[i].samples[j] << ',';
            burst_file_ << '\n';

            cat_file_ << c.cas[i].header.stamp.sec << ','
                      << c.cas[i].header.stamp.nsec << ','
                      << c.cas[i].x << ',' << c.cas[i].y << ",\n";
        }
    }
    else
        printf("Unable to open file!\n");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cat_creator");
    CatCreator cat_creator;
    return 0;
}
