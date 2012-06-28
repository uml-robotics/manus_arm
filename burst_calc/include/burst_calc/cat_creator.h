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
#include "burst_calc/burst.h"
#include "burst_calc/cat.h"
#include <fstream>

class CatCreator
{
public:
    CatCreator() {}
    ~CatCreator();
    void init(bool save_to_file, char* burst_file, char* cat_file);

private:
    void callback(const burst_calc::burst::ConstPtr& b);
    void initFile(char* burst_file, char* cat_file);
    void toFile(const burst_calc::burst& b, const burst_calc::cat& c);

    ros::NodeHandle n_;
    ros::Subscriber burst_sub_;
    ros::Publisher cat_pub_;
    std::ofstream burst_file_;
    std::ofstream cat_file_;
    bool save_to_file_;
};

#endif
