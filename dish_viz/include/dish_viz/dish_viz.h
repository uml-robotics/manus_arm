/*
 * dish_viz.h
 * Copyright 2013 University of Massachusetts Lowell
 * Authors: Abraham Shultz, Jonathan Hasenzahl
 */

#ifndef DISH_VIZ_H
#define DISH_VIZ_H

#include "ros/ros.h"
#include "dish_viz/DishVisualizer.h"
#include "neuro_recv/dish_state.h"
#include "burst_calc/ca.h"
#include "burst_calc/burst.h"
#include "burst_calc/ranges.h"
#include <queue>
#include <string>

/*!
 * \brief Node for visualing dish activity
 *
 * This node receives dish states from a receiver node and then plots that
 * activity on screen for the user to see. It also receives centers of activity
 * from the CAT creator node and plots those as well. This node is in sync with
 * the movements of the ARM.
 *
 * \copyright Copyright 2013 University of Massachusetts Lowell
 * \author Abraham Shultz
 * \author Jonathan Hasenzahl
 */
class DataHandler
{
public:
    DataHandler() { init(); }

private:
    void init();
    void getParams();
    void run();
    void plotCa();
    void updateMinMax(const neuro_recv::dish_state& d);
    void dishCallback(const neuro_recv::dish_state::ConstPtr& d);
    void caCallback(const burst_calc::ca::ConstPtr& c);
    void burstCallback(const burst_calc::burst::ConstPtr& b);
    void rangesCallback(const burst_calc::ranges::ConstPtr& r);

    ros::NodeHandle n_;
    ros::Subscriber dish_sub_;
    ros::Subscriber ca_sub_;
    ros::Subscriber burst_sub_;
    ros::Subscriber ranges_sub_;
    ros::ServiceClient time_client_;

    DishVisualizer dviz_;
    std::queue<neuro_recv::dish_state> queue_;
    std::queue<burst_calc::ca> cas_;
    int loop_rate_;
    int color_mode_;
    bool start_;
};

#endif
