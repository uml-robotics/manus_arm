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
    ~DataHandler() { if (loop_rate_) delete loop_rate_; }

private:
    void init();
    void getParams();
    void run();
    void update();
    void updateMinMax(const neuro_recv::dish_state& d);
    void dishCallback(const neuro_recv::dish_state::ConstPtr& d);
    void burstCallback(const burst_calc::burst::ConstPtr& b);
    void rangesCallback(const burst_calc::ranges::ConstPtr& r);

    ros::NodeHandle n_;
    ros::Subscriber dish_sub_;
    ros::Subscriber burst_sub_;
    ros::Subscriber ranges_sub_;
    ros::ServiceClient time_client_;
    ros::Rate* loop_rate_;

    DishVisualizer dviz_;
    std::queue<neuro_recv::dish_state> queue_;
    bool start_;
};

#endif
