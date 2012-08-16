// =============================================================================
// Name   : csv_recv.cpp
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Implements the ROS node "csv_receiver". This node receives data from a CSV
// file in order to test other nodes without needing a live link.
// =============================================================================

#include "neuro_recv/csv_recv.h"
#include <cstdio>

void CsvReceiver::init()
{
    if (getParams())
        file_.open(file_name_.c_str());
    else
        return;

    if (file_.is_open())
    {
        // Initialize the publishers
        initPubs();

        // Skip lines in the CSV file to account for headers and junk data
        // recorded during initialization
        for (int i = 0; i < skip_lines_; i++)
            file_.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        // Publish the buffer dishes
        publishBuffer();

        // Initialize the time stamp offset
        offset_ = ros::Time::now() - ros::Time(0);

        // Publish the rest of the dishes
        publish();

        file_.close();

        ROS_INFO("Reached end of CSV file");
    }
    else
        ROS_FATAL("Could not open file");
}

bool CsvReceiver::getParams()
{
    // Get file_name parameter
    std::string file_name;
    std::ifstream file;
    if (!n_.getParam("csv_file_path", file_name))
    {
        ROS_FATAL("Could not load csv_file_path parameter");
        return false;
    }

    // Get do_volt_distr parameter
    if (!n_.getParam("do_volt_distr", do_volt_distr_))
    {
        ROS_ERROR("Could not load do_volt_distr parameter, default is true");
        do_volt_distr_ = true;
    }

    // Get do_burst_calc parameter
    if (!n_.getParam("do_burst_calc", do_burst_calc_))
    {
        ROS_ERROR("Could not load do_burst_calc parameter, default is true");
        do_burst_calc_ = true;
    }

    // Get skip_lines parameter
    if (!n_.getParam("csv_skip_lines", skip_lines_))
    {
        ROS_ERROR("Could not load skip_lines parameter, default is 1");
        skip_lines_ = 1;
    }

    // Get buffer_size parameter
    if (!n_.getParam("buffer_size", buffer_size_))
    {
        ROS_ERROR("Could not load buffer_size parameter, default is 1000");
        buffer_size_ = 1000;
    }

    // Get loop rate parameter
    int loop_rate;
    if (!n_.getParam("loop_rate", loop_rate))
    {
        ROS_ERROR("Could not load loop_rate parameter, default is 200");
        loop_rate = 200;
    }
    loop_rate_ = new ros::Rate(loop_rate);

    return true;
}

void CsvReceiver::initPubs()
{
    ROS_INFO("Waiting for subscribers...");

    if (do_volt_distr_)
    {
        // Advertise and wait for a subscriber
        dish_pub_volt_ = n_.advertise<neuro_recv::dish_state>("dish_states_to_volt_distr",
                                                              1000);
        while (dish_pub_volt_.getNumSubscribers() < 1 && ros::ok());
    }

    if (do_burst_calc_)
    {
        // Advertise and wait for subscribers
        dish_pub_viz_ = n_.advertise<neuro_recv::dish_state>("dish_states_to_dish_viz",
                                                             1000);
        dish_pub_burst_ = n_.advertise<neuro_recv::dish_state>("dish_states_to_burst_creator",
                                                               1000);
        while (dish_pub_viz_.getNumSubscribers() < 1 &&
               dish_pub_burst_.getNumSubscribers() < 1 && ros::ok());
    }
}

void CsvReceiver::publishBuffer()
{
    ROS_INFO("Publishing buffer dishes...");

    std::string line;
    if (do_burst_calc_)
    {
        for (int i = 0; i < buffer_size_ && ros::ok(); i++)
        {
            getline(file_, line);
            dish_pub_burst_.publish(parse(line, false));
            loop_rate_->sleep();
        }
    }
    else
    {
        for (int i = 0; i < buffer_size_ && ros::ok(); i++)
            file_.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
}

void CsvReceiver::publish()
{
    ROS_INFO("Publishing dishes...");

    std::string line;
    while (getline(file_, line) && ros::ok())
    {
        neuro_recv::dish_state dish = parse(line, true);
        if (do_volt_distr_)
            dish_pub_volt_.publish(dish);
        if (do_burst_calc_)
        {
            dish_pub_viz_.publish(dish);
            dish_pub_burst_.publish(dish);
        }
        loop_rate_->sleep();
    }

    // Last dish lets the nodes know to finish up
    neuro_recv::dish_state end;
    end.last_dish = true;
    if (do_volt_distr_)
        dish_pub_volt_.publish(end);
    if (do_burst_calc_)
        dish_pub_burst_.publish(end);
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
    return 0;
}
