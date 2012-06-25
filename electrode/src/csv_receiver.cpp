// =============================================================================
// Name   : csv_receiver.cpp
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Implements the ROS node "csv_receiver". This node receives data from a CSV
// file in order to test other nodes without needing a live link.
// =============================================================================

#include "electrode/csv_receiver.h"
#include <fstream>
#include <cstdio>

void CsvReceiver::init(const char* file_name)
{
    std::ifstream file(file_name);

    if (file.is_open())
    {
        ROS_INFO("Reading from CSV file...");
        std::string line;

        // Skip the first 23 lines (header data) and the next 10,000 lines
        // (to account for initialization)
        for (int i = 0; i < 10023; i++)
            file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        ros::Publisher dish_state_pub = n_.advertise<electrode::dish_state>
                                                    ("dish_states", 1000);
        ros::Rate loop_rate(1000); // 1000 dishes per second

        // Wait for a subscriber to "dish_states" before publishing
        ROS_INFO("Waiting for subscriber...");
        while (dish_state_pub.getNumSubscribers() < 1 && ros::ok());
        ROS_INFO("Subscriber found. Continuing...");

        // Initialize the start time for timestamps
        start_time_ = ros::Time::now();

        while (getline(file, line) && ros::ok())
        {
            dish_state_pub.publish(parse(line));
            loop_rate.sleep();
        }

        ROS_INFO("Reached end of CSV file.");
        file.close();
    }
    else
        ROS_ERROR("Error: Cannot open %s", file_name);
}

const electrode::dish_state CsvReceiver::parse(const std::string& s)
{
    // Ignore first block of data, it is an index
    int n = 0;
    int pos = s.find(',', n) + 1;

    double total = 0.0;
    electrode::dish_state dish;
    ros::Time current_time = ros::Time::now();
    dish.header.stamp.sec = (current_time - start_time_).sec;
    dish.header.stamp.nsec = (current_time - start_time_).nsec;
    for (int i = 0; i < 60; i++)
    {
       n = s.find(',', pos) - pos;
       dish.samples[i] = atof(s.substr(pos, n).c_str());
       pos += n + 1;
       total += dish.samples[i];
    }

    return dish;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "csv_receiver");

    if (argc != 2)
    {
        ROS_ERROR("Error: CSV file name not specified.\n");
        return -1;
    }


    CsvReceiver csv_receiver;
    csv_receiver.init(argv[1]);
    return 0;
}
