/*
 * dish_viz.cpp
 *
 *  Created on: Jun 4, 2012
 *      Author: ams & Jonathan Hasenzahl
 */

#include "dish_viz.h"
#include "time_server/time_srv.h"

void DataHandler::init()
{
    // Initialize the visualizer
    dviz_.init();

    // Initialize the client and subscribers
    time_client_ = n_.serviceClient<time_server::time_srv>("time_service");

    dish_sub_ = n_.subscribe("dish_states_to_dish_viz", 1000,
                             &DataHandler::dishCallback, this);
    burst_sub_ = n_.subscribe("bursts_to_dish_viz", 1, &DataHandler::burstCallback,
                              this);
    ranges_sub_ = n_.subscribe("ranges_to_dish_viz", 1, &DataHandler::rangesCallback,
                               this);

    // Wait for start
    ROS_INFO("Waiting to start...");
    start_ = false;
    while (!start_ && ros::ok())
        ros::spinOnce();

    // Run the visualizer
    run();
}

void DataHandler::getParams()
{
    // Get loop rate parameter
    int loop_rate;
    if (!n_.getParam("loop_rate", loop_rate))
    {
        ROS_ERROR("Could not load loop_rate parameter, default is 200");
        loop_rate = 200;
    }
    loop_rate_ = new ros::Rate(loop_rate);
}

void DataHandler::run()
{
    while (!queue_.empty() && ros::ok())
    {
        ros::spinOnce();
        update();
        loop_rate_->sleep();
    }
}

void DataHandler::update()
{
    if (!queue_.empty())
    {
        neuro_recv::dish_state d = queue_.front();
        //printf("[%.3f][%.3f] Queue size: %d\n", ros::Time::now().toSec(),
        //           d.header.stamp.toSec(), static_cast<int>(queue_.size()));
        for(int i = 0; i < 60; i++)
            dviz_.update(i, static_cast<double>(d.samples[i]));
        queue_.pop();
    }
}

void DataHandler::updateMinMax(const neuro_recv::dish_state& d)
{
    for (int i = 0; i < 60; i++)
    {
        if (d.samples[i] > dviz_.getMaxVolt(i))
        {
            dviz_.setMaxVolt(i, d.samples[i]);
            ROS_INFO("Range correction: max of channel %d set to %f", i,
                     d.samples[i]);
        }
        else if (d.samples[i] < dviz_.getMinVolt(i))
        {
            dviz_.setMinVolt(i, d.samples[i]);
            ROS_INFO("Range correction: min of channel %d set to %f", i,
                     d.samples[i]);
        }
    }
}

void DataHandler::dishCallback(const neuro_recv::dish_state::ConstPtr &d)
{
    updateMinMax(*d);
    queue_.push(*d);
}

void DataHandler::burstCallback(const burst_calc::burst::ConstPtr &b)
{
    start_ = true;
}

void DataHandler::rangesCallback(const burst_calc::ranges::ConstPtr& r)
{
    dviz_.setVoltRanges(r->baselines, r->thresholds, r->min_volts, r->max_volts);
    for (int i = 0; i < 60; i++)
        printf("%d: Min[%f] Base[%f] Tresh[%f] Max[%f]\n", i, r->min_volts[i],
               r->baselines[i], r->thresholds[i], r->max_volts[i]);
    ROS_INFO("Min/max/baseline ranges set");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dish_viz");
	DataHandler dh;
	return 0;
}

