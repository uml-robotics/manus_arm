/*
 * dish_viz.cpp
 * Copyright 2013 University of Massachusetts Lowell
 * Authors: Abraham Shultz, Jonathan Hasenzahl
 */

#include "dish_viz/dish_viz.h"
#include "time_server/time_srv.h"

/*!
 * \brief Initializes the node
 *
 * Gets server parameters, initializes the visualizer, initializes subscribers,
 * and runs the spin loop.
 */
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

/*!
 * \brief Gets server parameters
 */
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

/*!
 * \brief Main spin loop
 *
 * Spins until the queue is empty, ROS shuts down, or Ctrl+C is pressed.
 */
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

/*!
 * \brief Plots a center of activity on the visualizer
 */
void DataHandler::plotCa()
{
    if (!cas_.empty())
    {
        time_server::time_srv ca_check;
        ca_check.request.target = cas_.front().header.stamp;

        if (time_client_.call(ca_check))
        {
            printf("CA Time - Server Time = %f\n", ca_check.response.delta.toSec());

            // Send the CA to the visualizer if its equal to server time, with
            // a small room for error. If the CA is more than 0.1s behind server
            // time, then something has gone wrong.
            if (ca_check.response.delta <= ros::Duration(0.05) &&
                ca_check.response.delta >= ros::Duration(-0.1))
            {
                dviz_.updateCa(cas_.front());
                cas_.pop();
            }
            else if (ca_check.response.delta < ros::Duration(-0.1))
            {
                ROS_ERROR("CA time is behind visualizer time and will not display");
                cas_.pop();
            }
        }
        else
            ROS_ERROR("Time server is not responding");
    }
}

/*!
 * \brief Updates minimum and maximum values for each channel
 *
 * This is a self-correcting feature of the visualizer, or else colors could
 * become inaccurate the longer the node runs.
 *
 * \param d the dish state used for the update
 */
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

/*!
 * \brief Callback for dish state messages
 *
 * This method is called automatically when the node receives a dish state
 * message. The minimum and maximum voltages are updated, and the dish state
 * is pushed onto the display queue.
 *
 * \param d the received message
 */
void DataHandler::dishCallback(const neuro_recv::dish_state::ConstPtr &d)
{
    updateMinMax(*d);
    queue_.push(*d);
    //printf("Queue size: %d\n", static_cast<int>(queue_.size()));
}

/*!
 * \brief Callback for center of activity messages
 *
 * This method is called automatically when the node receives a CA message. The
 * CA is pushed onto the display queue.
 *
 * \param c the received message
 */
void DataHandler::caCallback(const burst_calc::ca::ConstPtr& c)
{
    cas_.push(*c);
}

/*!
 * \brief Callback for burst messages
 *
 * This method is called automatically when the node receives a burst message.
 * This should only happen once. This is just a signal to tell the visualizer
 * to start playback.
 *
 * \param r the received message
 */
void DataHandler::burstCallback(const burst_calc::burst::ConstPtr &b)
{
    start_ = true;
}

/*!
 * \brief Callback for ranges messages
 *
 * This method is called automatically when the node receives a ranges message.
 * This should only happen once. The range values are used for calculating the
 * display colors of voltages on each channel.
 *
 * \param r the received message
 */
void DataHandler::rangesCallback(const burst_calc::ranges::ConstPtr& r)
{
    dviz_.setVoltRanges(r->baselines, r->thresholds, r->min_volts, r->max_volts);

    /*
    for (int i = 0; i < 60; i++)
        printf("%d: Min[%.5f] Base[%.5f] Thresh[%.5f] Max[%.5f]\n", i, r->min_volts[i],
               r->baselines[i], r->thresholds[i], r->max_volts[i]);
    */
}

/*!
 * \brief Creates an instance of the node
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "dish_viz");
	DataHandler dh;
	return 0;
}

