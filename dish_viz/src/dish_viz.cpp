/*
 * dish_viz.cpp
 *
 *  Created on: Jun 4, 2012
 *      Author: ams & Jonathan Hasenzahl
 */

#include "dish_viz/dish_viz.h"
#include "time_server/time_srv.h"

void DataHandler::init()
{
    // Get parameters
    getParams();

    // Initialize the visualizer
    dviz_.init(color_mode_);

    // Initialize the client and subscribers
    time_client_ = n_.serviceClient<time_server::time_srv>("time_service");

    dish_sub_ = n_.subscribe("dish_states_to_dish_viz", 1000,
                             &DataHandler::dishCallback, this);
    ca_sub_ = n_.subscribe("cas", 1000, &DataHandler::caCallback, this);
    burst_sub_ = n_.subscribe("bursts_to_dish_viz", 1, &DataHandler::burstCallback,
                              this);
    ranges_sub_ = n_.subscribe("ranges_to_dish_viz", 1, &DataHandler::rangesCallback,
                               this);

    // Wait for start signal
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
    if (!n_.getParam("loop_rate", loop_rate_))
    {
        ROS_WARN("Could not load loop_rate parameter: default is 200");
        loop_rate_ = 200;
    }

    // Get color mode parameter
    if (!n_.getParam("visualizer_color_mode", color_mode_))
    {
        ROS_WARN("Could not load visualizer_color_mode parameter: default is 0");
        color_mode_ = 0;
    }
}

void DataHandler::run()
{
    ROS_INFO("Running visualizer");

    ros::Rate loop_rate(loop_rate_);

    while (!queue_.empty() && ros::ok())
    {
        ros::spinOnce();

        // Update the visualizer channels
        neuro_recv::dish_state d = queue_.front();
        //printf("Queue size: %d\n", static_cast<int>(queue_.size()));
        for (int i = 0; i < 60; i++)
            dviz_.update(i, static_cast<double>(d.samples[i]));
        queue_.pop();

        // Plot CA if conditions are met
        plotCa();

        loop_rate.sleep();
    }

    ROS_INFO("Visualizer queue empty: shutting down");
}

void DataHandler::plotCa()
{
    if (!cas_.empty())
    {
        time_server::time_srv ca_check;
        ca_check.request.target = cas_.front().header.stamp;

        if (time_client_.call(ca_check))
        {
            printf("CA time     : %f\n", ca_check.request.target.toSec());
            printf("Server time : %f\n", ca_check.response.actual.toSec());
            printf("Delta       : %f\n", ca_check.response.delta.toSec());

            if (ca_check.response.delta <= ros::Duration(0.1) &&
                ca_check.response.delta >= ros::Duration(-0.1))
            {
                dviz_.updateCa(cas_.front());
                cas_.pop();
            }
            else if (ca_check.response.delta < ros::Duration(-0.1))
            {
                ROS_ERROR("CA is behind visualizer and will not display");
                cas_.pop();
            }
        }
        else
            ROS_ERROR("Time server is not responding");
    }
}

void DataHandler::updateMinMax(const neuro_recv::dish_state& d)
{
    for (int i = 0; i < 60; i++)
    {
        if (d.samples[i] > dviz_.getMaxVolt(i))
        {
            dviz_.setMaxVolt(i, d.samples[i]);
            //printf("Range correction: max of channel %d set to %f\n", i,
            //       d.samples[i]);
        }
        else if (d.samples[i] < dviz_.getMinVolt(i))
        {
            dviz_.setMinVolt(i, d.samples[i]);
            //printf("Range correction: min of channel %d set to %f\n", i,
            //       d.samples[i]);
        }
    }
}

void DataHandler::dishCallback(const neuro_recv::dish_state::ConstPtr &d)
{
    updateMinMax(*d);
    queue_.push(*d);
    //printf("Queue size: %d\n", static_cast<int>(queue_.size()));
}

void DataHandler::caCallback(const burst_calc::ca::ConstPtr& c)
{
    cas_.push(*c);
}

void DataHandler::burstCallback(const burst_calc::burst::ConstPtr &b)
{
    start_ = true;
}

void DataHandler::rangesCallback(const burst_calc::ranges::ConstPtr& r)
{
    dviz_.setVoltRanges(r->baselines, r->thresholds, r->min_volts, r->max_volts);

    /*
    for (int i = 0; i < 60; i++)
        printf("%d: Min[%.5f] Base[%.5f] Thresh[%.5f] Max[%.5f]\n", i, r->min_volts[i],
               r->baselines[i], r->thresholds[i], r->max_volts[i]);
    */
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dish_viz");
	DataHandler dh;
	return 0;
}

