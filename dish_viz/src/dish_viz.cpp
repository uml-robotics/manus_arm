/*
 * dish_viz.cpp
 *
 *  Created on: Jun 4, 2012
 *      Author: ams & Jonathan Hasenzahl
 */

#include "dish_viz.h"


void DataHandler::init(int rate, const std::string& dish_topic,
                       const std::string& burst_topic,
                       const std::string& ranges_topic)
{
    dViz.init();
    dish_sub_ = n_.subscribe(dish_topic.c_str(), 1000,
                             &DataHandler::dishCallback, this);
    burst_sub_ = n_.subscribe(burst_topic.c_str(), 1,
                              &DataHandler::burstCallback, this);
    ranges_sub_ = n_.subscribe(ranges_topic.c_str(), 1,
                               &DataHandler::rangesCallback, this);
    start_ = false;
    while (!start_)
        ros::spinOnce();

    ros::Rate loop_rate(rate);
    ros::Time start = ros::Time::now();

    while (ros::ok() & !queue_.empty())
    {
        ros::spinOnce();
        update();
        loop_rate.sleep();
    }

    ROS_INFO("Duration: %.3fs", (ros::Time::now() - start).toSec());
}

void DataHandler::update()
{
    if (!queue_.empty())
    {
        neuro_recv::dish_state d = queue_.front();
        for(int i = 0; i < 60; i++)
            dViz.update(i, static_cast<double>(d.samples[i]));
        queue_.pop();
    }
}

void DataHandler::dishCallback(const neuro_recv::dish_state::ConstPtr &d)
{
    queue_.push(*d);
}

void DataHandler::burstCallback(const burst_calc::burst::ConstPtr &b)
{
    start_ = true;
}

void DataHandler::rangesCallback(const burst_calc::ranges::ConstPtr& r)
{
    dViz.setVoltRanges(r->baselines, r->min_volts, r->max_volts);
    ROS_INFO("rangesCallback called");
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "dish_viz");


	// Declare variables that can be modified by launch file or command line.
	int rate;
	string dish_topic;
	string burst_topic;
	string ranges_topic;

	// Initialize node parameters from launch file or command line.
	// Use a private node handle so that multiple instances of the node can be run simultaneously
	// while using different parameters.
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.param("rate", rate, 200);
	private_node_handle_.param("dish_topic", dish_topic, string("fwd_dish_states"));
	private_node_handle_.param("burst_topic", burst_topic, string("fwd_bursts"));
	private_node_handle_.param("ranges_topic", ranges_topic, string("ranges"));

	DataHandler dh;
	dh.init(rate, dish_topic, burst_topic, ranges_topic);

	return 0;
}

