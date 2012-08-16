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

    if (file.is_open())
    {
        // Get skip_lines parameter
        int skip_lines;
        if (!n_.getParam("csv_skip_lines", skip_lines))
        {
            ROS_ERROR("Could not load buffer_size parameter, default will be used");
            skip_lines = 0;
        }

        // Skip lines in the CSV file to account for headers and junk data
        // recorded during initialization
        for (int i = 0; i < skip_lines; i++)
            file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        ros::Publisher dish_state_pub = n_.advertise<neuro_recv::dish_state>
                                                    ("dish_states", 1000);

        // Wait for a subscriber to "dish_states" before publishing
        ROS_INFO("Waiting for subscriber...");
        while (dish_state_pub.getNumSubscribers() < 1 && ros::ok());
        ROS_INFO("Subscriber found. Continuing...");

        // Get buffer_size parameter
        int buffer_size;
        if (!n_.getParam("buffer_size", buffer_size))
        {
            ROS_ERROR("Could not load buffer_size parameter, default will be used");
            buffer_size = 1000;
        }

        // Get loop rate parameter
        int rate;
        if (!n_.getParam("loop_rate", rate))
        {
            ROS_ERROR("Could not load loop_rate parameter, default will be used");
            rate = 200;
        }
        ros::Rate loop_rate(rate);

        // Publish the buffer dishes without a timestamp
        std::string line;
        int num_dishes = 0;
        while (num_dishes++ < buffer_size && ros::ok())
        {
            getline(file, line);
            dish_state_pub.publish(parse(line, false));
            loop_rate.sleep();
        }

        // Initialize the timestamp offset and seed the time server
        offset_ = ros::Time::now() - ros::Time(0);

        // Publish the rest of the dishes with a timestamp
        while (getline(file, line) && ros::ok())
        {
            dish_state_pub.publish(parse(line, true));
            loop_rate.sleep();
        }

        // Last dish flag
        neuro_recv::dish_state end;
        end.last_dish = true;
        dish_state_pub.publish(end);

        ROS_INFO("Reached end of CSV file");
        file.close();
    }
    else
        ROS_FATAL("Could not open file");
}

const neuro_recv::dish_state CsvReceiver::parse(const std::string& s,
                                                bool record_time)
{
    // Ignore first block of data, it is an index
    int n = 0;
    int pos = s.find(',', n) + 1;

    neuro_recv::dish_state dish;
    if (record_time)
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
