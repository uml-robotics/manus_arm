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
    dViz.init();
    time_client_ = n_.serviceClient<time_server::time_srv>("time_service");

    dish_sub_ = n_.subscribe("fwd_dish_states", 1000,
                             &DataHandler::dishCallback, this);
    burst_sub_ = n_.subscribe("fwd_bursts", 1, &DataHandler::burstCallback,
                              this);
    ranges_sub_ = n_.subscribe("ranges", 1, &DataHandler::rangesCallback,
                               this);

    // Get loop rate parameter
    int rate;
    if (!n_.getParam("loop_rate", rate))
    {
        ROS_ERROR("Could not load loop_rate parameter, default will be used");
        rate = 200;
    }

    // Get buffer size parameter
    if (!n_.getParam("buffer_size", buffer_size_))
    {
        ROS_ERROR("Could not load buffer_size parameter, default will be used");
        buffer_size_ = 1000;
    }

    // Wait for signal to start (call to rangesCallback)
    ROS_INFO("Waiting to start...");
    start_ = false;
    while (!start_)
        ros::spinOnce();
    ROS_INFO("Starting...");

    ros::Rate loop_rate(rate);
    ros::Time start = ros::Time::now();

    int step = 0;
    while (ros::ok() & !queue_.empty())
    {
        ros::spinOnce();
        if (++step == 1000)
        {
            time_server::time_srv check;
            check.request.target = queue_.front().header.stamp;
            time_client_.call(check);
            printf("Dish start time : %f\n", check.request.target.toSec());
            printf("Server time     : %f\n", check.response.actual.toSec());
            printf("Delta           : %f\n\n", check.response.delta.toSec());
            step = 0;
        }
        update();
        loop_rate.sleep();
    }

    ROS_INFO("Duration: %fs", (ros::Time::now() - start).toSec());
}

void DataHandler::update()
{
    if (!queue_.empty())
    {
        neuro_recv::dish_state d = queue_.front();
        //printf("[%.3f][%.3f] Queue size: %d\n", ros::Time::now().toSec(),
        //           d.header.stamp.toSec(), static_cast<int>(queue_.size()));
        for(int i = 0; i < 60; i++)
            dViz.update(i, static_cast<double>(d.samples[i]));
        queue_.pop();
    }
}

void DataHandler::dishCallback(const neuro_recv::dish_state::ConstPtr &d)
{
    // Don't start storing dishes in the queue until the buffer is done
    if (dishes_received_ < buffer_size_)
        dishes_received_++;
    else
        queue_.push(*d);
}

void DataHandler::burstCallback(const burst_calc::burst::ConstPtr &b)
{
    start_ = true;
}

void DataHandler::rangesCallback(const burst_calc::ranges::ConstPtr& r)
{
    dViz.setVoltRanges(r->baselines, r->min_volts, r->max_volts);
    for (int i = 0; i < 60; i++)
        printf("%d: Min[%f] Base[%f] Max[%f]\n", i, r->min_volts[i],
               r->baselines[i], r->max_volts[i]);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dish_viz");
	DataHandler dh;
	dh.init();
	return 0;
}

