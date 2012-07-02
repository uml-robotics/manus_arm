// =============================================================================
// Name   : csv_recv.h
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Header file for the ROS node "csv_receiver".
// =============================================================================

#ifndef CSV_RECEIVER_H_
#define CSV_RECEIVER_H_

#include "ros/ros.h"
#include "neuro_recv/dish_state.h"
#include <string>

class CsvReceiver
{
public:
    CsvReceiver() {}
    void init(const char* file_name);

private:
    const neuro_recv::dish_state parse(const std::string& s);

    ros::NodeHandle n_;
    ros::Duration offset_;
};

#endif
