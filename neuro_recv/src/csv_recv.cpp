// =============================================================================
// Name   : csv_recv.cpp
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Implements the ROS node "csv_receiver". This node receives data from a CSV
// file in order to test other nodes without needing a live link.
// =============================================================================

#include "neuro_recv/csv_recv.h"
#include <fstream>
#include <cstdio>

void CsvReceiver::init()
{
    // Get file name parameter
    std::string file_name;
    std::ifstream file;
    if (n_.getParam("csv_file_path", file_name))
    {
        file.open(file_name.c_str());
    }
    else
    {
        ROS_FATAL("Could not load csv_file_path parameter");
        return;
    }

    // Get loop rate parameter
    int rate;
    if (!n_.getParam("loop_rate", rate))
    {
        ROS_ERROR("Could not load loop_rate parameter, default will be used");
        rate = 200;
    }

    if (file.is_open())
    {
        std::string line;

        // Skip the first 23 lines (header data) and the next 10,000 lines
        // (to account for initialization)
        for (int i = 0; i < 10023; i++)
            file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        ros::Publisher dish_state_pub = n_.advertise<neuro_recv::dish_state>
                                                    ("dish_states", 1000);

        // Wait for a subscriber to "dish_states" before publishing
        ROS_INFO("Waiting for subscriber...");
        while (dish_state_pub.getNumSubscribers() < 1 && ros::ok());
        ROS_INFO("Subscriber found. Continuing...");

        // Publish at a rate of 200 dishes per second, which is 5 times slower
        // than the actual rate of 1000 dishes per second
        ros::Rate loop_rate(rate);

        // Initialize the timestamp offset
        offset_ = ros::Time::now() - ros::Time(0);

        while (getline(file, line) && ros::ok())
        {
            dish_state_pub.publish(parse(line));
            loop_rate.sleep();
        }
        ROS_INFO("Reached end of CSV file");
        file.close();
    }
    else
        ROS_FATAL("Could not open file");
}

const neuro_recv::dish_state CsvReceiver::parse(const std::string& s)
{
    // Ignore first block of data, it is an index
    int n = 0;
    int pos = s.find(',', n) + 1;

    neuro_recv::dish_state dish;
    dish.header.stamp = ros::Time::now() - offset_;

    for (int i = 0; i < 60; i++)
    {
       n = s.find(',', pos) - pos;
       dish.samples[i] = atof(s.substr(pos, n).c_str());
       pos += n + 1;
    }

    return dish;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "csv_receiver");
    CsvReceiver csv_receiver;
    csv_receiver.init();
    return 0;
}
