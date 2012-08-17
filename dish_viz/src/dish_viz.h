/*
 * Visualizer.h
 *
 *  Created on: Mar 29, 2012
 *      Author: ams & Jonathan Hasenzahl
 */

#ifndef DISH_VIZ_H
#define DISH_VIZ_H

#include "ros/ros.h"
#include "DishVisualizer.h"
#include "neuro_recv/dish_state.h"
#include "burst_calc/burst.h"
#include "burst_calc/ranges.h"
#include <queue>
#include <string>

class DataHandler
{
public:
    DataHandler() { init(); }

private:
    void init();
    void getParams();
    void run();
    void updateMinMax(const neuro_recv::dish_state& d);
    void dishCallback(const neuro_recv::dish_state::ConstPtr& d);
    void burstCallback(const burst_calc::burst::ConstPtr& b);
    void rangesCallback(const burst_calc::ranges::ConstPtr& r);

    ros::NodeHandle n_;
    ros::Subscriber dish_sub_;
    ros::Subscriber burst_sub_;
    ros::Subscriber ranges_sub_;
    ros::ServiceClient time_client_;

    DishVisualizer dviz_;
    std::queue<neuro_recv::dish_state> queue_;
    int loop_rate_;
    bool start_;
};

#endif
