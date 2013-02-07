/*
 * csv_recv.h
 * Copyright 2013 University of Massachusetts Lowell
 * Author: Jonathan Hasenzahl
 */

#ifndef CSV_RECEIVER_H_
#define CSV_RECEIVER_H_

#include "ros/ros.h"
#include "neuro_recv/dish_state.h"
#include <string>
#include <fstream>

/*!
 * \brief Node for publishing dish states from a CSV file
 *
 * This node receives data from a CSV file in order to test other nodes without
 * needing a live link. The file path of the CSV file is a server parameter.
 *
 * \copyright Copyright 2013 University of Massachusetts Lowell
 * \author Jonathan Hasenzahl
 */
class CsvReceiver
{
public:
    CsvReceiver() { init(); }

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
    ros::Duration offset_;

    std::string file_name_;
    std::ifstream file_;

    bool do_volt_distr_;
    bool do_burst_calc_;
    int skip_lines_;
    int buffer_size_;
    int loop_rate_;
};

#endif
