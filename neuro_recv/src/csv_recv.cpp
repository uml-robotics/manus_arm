// =============================================================================
// Name   : csv_recv.cpp
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Implements the ROS node "csv_receiver". This node receives data from a CSV
// file in order to test other nodes without needing a live link.
// =============================================================================

#include "neuro_recv/csv_recv.h"
#include "time_server/time_srv.h"
#include <fstream>
#include <cstdio>

void CsvReceiver::init()
{
    client_ = n_.serviceClient<time_server::time_srv>("time_service");

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
        // Skip the first 23 lines (header data) and the next 10,000 lines
        // (to account for initialization)
        for (int i = 0; i < 10023; i++)
            file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        ros::Publisher dish_state_pub = n_.advertise<neuro_recv::dish_state>
                                                    ("dish_states", 1000);

        // Wait for a subscriber to "dish_states" before publishing
        ROS_INFO("Waiting for subscriber...");
        //while (dish_state_pub.getNumSubscribers() < 1 && ros::ok());
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
        time_server::time_srv seed;
        seed.request.target = ros::Time(0) + offset_;
        if (client_.call(seed))
        {
            ROS_INFO("Time server seeded successfully");
            // Publish the rest of the dishes with a timestamp
            while (getline(file, line) && ros::ok())
            {
                dish_state_pub.publish(parse(line, true));
                loop_rate.sleep();
            }

            ROS_INFO("Reached end of CSV file");

            time_server::time_srv test;
            test.request.target = ros::Time::now() - offset_;
            client_.call(test);
            printf("Local time  : %f\n", test.request.target.toSec());
            printf("Server time : %f\n", test.response.actual.toSec());
            printf("Delta       : %f\n", test.response.delta.toSec());
        }
        else
            ROS_FATAL("Time server did not respond");


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
