/*
 * teleop_arm_dish.h
 * Copyright 2013 University of Massachusetts Lowell
 */

#ifndef TELEOP_ARM_DISH_H_
#define TELEOP_ARM_DISH_H_

#include "ros/ros.h"
#include "burst_calc/cat.h"
#include <queue>

/*!
 * \brief Node for ARM teleop from the neuron dish
 *
 * This node receives CAT (center of activity trajectory) data from the CAT
 * creator node and then creates and publishes movement commands. The operation
 * of the node is autonomous; after the object has been created, the
 * node will run with no further operation.
 *
 * \copyright 2013 University of Massachusetts Lowell
 * \author Jonathan Hasenzahl
 */
class TeleopArmDish
{
public:
    TeleopArmDish();

private:
    void init();
    void getParams();
    void run();
    void callback(const burst_calc::cat::ConstPtr& c);
    void publishCartesianMove();
    void publishConstantMove();
    double getArmCoord(double coord);

    ros::NodeHandle n_;
    ros::Subscriber cat_sub_;
    ros::Publisher cmd_pub_;
    ros::ServiceClient time_client_;

    std::queue<burst_calc::cat> queue_;

    int speed_;
    double arm_safe_range_;
    double max_range_from_midpoint_;
};

#endif
