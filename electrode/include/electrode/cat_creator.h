// =============================================================================
// Name   : cat_creator.h
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Header file for the ROS node "cat_creator".
// =============================================================================

#ifndef CAT_CREATOR_H_
#define CAT_CREATOR_H_

#include "ros/ros.h"
#include "electrode/burst.h"
#include "electrode/cat.h"
#include <fstream>

class CatCreator
{
public:
    CatCreator();
    ~CatCreator()
    {
        if (burst_file_.is_open())
            burst_file_.close();
        if (cat_file_.is_open())
            cat_file_.close();
    }

private:
    void callback(const electrode::burst::ConstPtr& b);
    void toFile(const electrode::burst& b, const electrode::cat& c);

    ros::NodeHandle n_;
    ros::Subscriber burst_sub_;
    ros::Publisher cat_pub_;
    std::ofstream burst_file_;
    std::ofstream cat_file_;
};

#endif
