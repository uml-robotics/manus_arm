/*
 * dish_viz.cpp
 *
 *  Created on: Jun 4, 2012
 *      Author: ams & Jonathan Hasenzahl
 */

#include "dish_viz.h"


void DataHandler::init()
{
    dViz.init();

    dish_sub_ = n_.subscribe("fwd_dish_states", 1000,
                             &DataHandler::dishCallback, this);
    burst_sub_ = n_.subscribe("fwd_bursts", 1, &DataHandler::burstCallback,
                              this);
    ranges_sub_ = n_.subscribe("ranges", 1, &DataHandler::rangesCallback,
                               this);

    // Wait for signal to start (call to rangesCallback)
    ROS_INFO("Waiting to start...");
    start_ = false;
    while (!start_)
        ros::spinOnce();
    ROS_INFO("Starting...");

    // Get loop rate parameter
    int rate;
    if (!n_.getParam("loop_rate", rate))
    {
        ROS_ERROR("Could not load loop_rate parameter, default will be used");
        rate = 200;
    }

    ros::Rate loop_rate(rate);
    ros::Time start = ros::Time::now();

    while (ros::ok() & !queue_.empty())
    {
        ros::spinOnce();
        update();
        loop_rate.sleep();
    }

    ROS_INFO("Playback duration: %.3fs", (ros::Time::now() - start).toSec());
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
    //ROS_INFO("dishCallback called");
}

void DataHandler::burstCallback(const burst_calc::burst::ConstPtr &b)
{
    start_ = true;
}

void DataHandler::rangesCallback(const burst_calc::ranges::ConstPtr& r)
{
    dViz.setVoltRanges(r->baselines, r->min_volts, r->max_volts);
    for (int i = 0; i < 60; i++)
        printf("%d: Min[%.3f] Base[%.3f] Max[%.3f]\n", i, r->min_volts[i],
               r->baselines[i], r->max_volts[i]);
    //ROS_INFO("rangesCallback called");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dish_viz");
	DataHandler dh;
	dh.init();
	return 0;
}

