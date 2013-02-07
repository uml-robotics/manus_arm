/*
 * csv_recv.cpp
 * Copyright 2013 University of Massachusetts Lowell
 * Author: Jonathan Hasenzahl
 */

#include "neuro_recv/csv_recv.h"
#include <cstdio>

/*!
 * \brief Initializes and runs the node
 *
 * Gets server parameters, attempts to open the CSV file, initializes
 * publishers, reads and parses file data, and publishes the dish states.
 */
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

/*!
 * \brief Gets server parameters
 * \return true if csv_file_path exists as a parameter, false otherwise
 */
bool CsvReceiver::getParams()
{
    // Get file_name parameter
    if (!n_.getParam("csv_file_path", file_name_))
    {
        ROS_FATAL("Could not load csv_file_path parameter");
        return false;
    }

    // Get do_volt_distr parameter
    if (!n_.getParam("do_volt_distr", do_volt_distr_))
    {
        ROS_WARN("Could not load do_volt_distr parameter, default is true");
        do_volt_distr_ = true;
    }

    // Get do_burst_calc parameter
    if (!n_.getParam("do_burst_calc", do_burst_calc_))
    {
        ROS_WARN("Could not load do_burst_calc parameter, default is true");
        do_burst_calc_ = true;
    }

    // Get skip_lines parameter
    if (!n_.getParam("csv_skip_lines", skip_lines_))
    {
        ROS_WARN("Could not load skip_lines parameter, default is 1");
        skip_lines_ = 1;
    }

    // Get buffer_size parameter
    if (!n_.getParam("buffer_size", buffer_size_))
    {
        ROS_WARN("Could not load buffer_size parameter, default is 1000");
        buffer_size_ = 1000;
    }

    // Get loop rate parameter
    if (!n_.getParam("loop_rate", loop_rate_))
    {
        ROS_WARN("Could not load loop_rate parameter, default is 200");
        loop_rate_ = 200;
    }

    return true;
}

/*!
 * \brief Initializes ROS publishers
 */
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

/*!
 * \brief Publishes dish states used to buffer the burst creator
 *
 * These dish states are only sent to the burst_creator node. This method must
 * be called before publish.
 */
void CsvReceiver::publishBuffer()
{
    ROS_INFO("Publishing buffer dishes...");

    ros::Rate loop_rate(loop_rate_);
    std::string line;

    if (do_burst_calc_)
    {
        for (int i = 0; i < buffer_size_ && ros::ok(); i++)
        {
            getline(file_, line);
            dish_pub_burst_.publish(parse(line, false));
            loop_rate.sleep();
        }
    }
    else
    {
        for (int i = 0; i < buffer_size_ && ros::ok(); i++)
            file_.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
}

/*!
 * \brief Publishes all remaining dish states
 *
 * These dish states are sent to burst_creator, dish_viz, and volt_distr. This
 * method must be called after publishBuffer.
 */
void CsvReceiver::publish()
{
    ROS_INFO("Publishing dishes...");

    ros::Rate loop_rate(loop_rate_);
    std::string line;

    if (do_volt_distr_ && do_burst_calc_)
    {
        while (getline(file_, line) && ros::ok())
        {
            neuro_recv::dish_state dish = parse(line, true);
            dish_pub_volt_.publish(dish);
            dish_pub_viz_.publish(dish);
            dish_pub_burst_.publish(dish);
            loop_rate.sleep();
        }
    }
    else if (do_volt_distr_)
    {
        while (getline(file_, line) && ros::ok())
        {
            neuro_recv::dish_state dish = parse(line, true);
            dish_pub_volt_.publish(dish);
            loop_rate.sleep();
        }
    }
    else if (do_burst_calc_)
    {
        while (getline(file_, line) && ros::ok())
        {
            neuro_recv::dish_state dish = parse(line, true);
            dish_pub_viz_.publish(dish);
            dish_pub_burst_.publish(dish);
            loop_rate.sleep();
        }
    }

    // Last dish lets the nodes know to finish up
    neuro_recv::dish_state end;
    end.last_dish = true;
    if (do_volt_distr_)
        dish_pub_volt_.publish(end);
    if (do_burst_calc_)
        dish_pub_burst_.publish(end);
}

/*!
 * \brief Parses a single line in a CSV file and returns a new dish state
 *
 * The input string must be 61 comma-separated values. The first value is an
 * index and is ignored. The remaining 60 values are voltages for each channel
 * in the MEA.
 *
 * \param s the input string
 * \param record_time whether or not to record the current time in the dish state
 * \return the new dish state
 */
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

/*!
 * \brief Creates an instance of the node
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "csv_receiver");
    CsvReceiver csv_receiver;
    return 0;
}
