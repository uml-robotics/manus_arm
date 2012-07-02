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
#include <queue>
#include <string>

class DataHandler
{
public:
    DataHandler() {}

    void init(int rate, const std::string& dish_topic,
              const std::string& burst_topic);
    void update();
    void dishCallback(const neuro_recv::dish_state::ConstPtr &d);
    void burstCallback(const burst_calc::burst::ConstPtr &b);

private:
    ros::NodeHandle n_;
    ros::Subscriber dish_sub_;
    ros::Subscriber burst_sub_;
    DishVisualizer dViz;
    std::queue<neuro_recv::dish_state> queue_;
    bool start_;
};

#endif
