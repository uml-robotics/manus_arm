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
#include <fstream>

class CsvReceiver
{
public:
    CsvReceiver() { init(); }
    ~CsvReceiver() { if (loop_rate_) delete loop_rate_; }

private:
    void init();
    bool getParams();
    void initPubs();
    void publishBuffer();
    void publish();
    const neuro_recv::dish_state parse(const std::string&, bool record_time);

    ros::NodeHandle n_;
    ros::Publisher dish_pub_volt_;
    ros::Publisher dish_pub_viz_;
    ros::Publisher dish_pub_burst_;
    ros::Rate* loop_rate_;
    ros::Duration offset_;

    std::string file_name_;
    std::ifstream file_;

    bool do_volt_distr_;
    bool do_burst_calc_;
    int skip_lines_;
    int buffer_size_;
};

#endif
