/*
 * cat_creator.h
 * Copyright 2013 University of Massachusetts Lowell
 * Author: Jonathan Hasenzahl
 */

#ifndef CAT_CREATOR_H_
#define CAT_CREATOR_H_

#include "ros/ros.h"
#include "burst_calc/burst.h"
#include "burst_calc/ranges.h"
#include "burst_calc/cat.h"
#include "burst_calc/ca.h"
#include "neuro_recv/dish_state.h"
#include <fstream>
#include <string>

const int X_COORD_[60] = {    2, 3, 4, 5, 6, 7,
                           1, 2, 3, 4, 5, 6, 7, 8,
                           1, 2, 3, 4, 5, 6, 7, 8,
                           1, 2, 3, 4, 5, 6, 7, 8,
                           1, 2, 3, 4, 5, 6, 7, 8,
                           1, 2, 3, 4, 5, 6, 7, 8,
                           1, 2, 3, 4, 5, 6, 7, 8,
                              2, 3, 4, 5, 6, 7     };
const int Y_COORD_[60] = {    1, 1, 1, 1, 1, 1,
                           2, 2, 2, 2, 2, 2, 2, 2,
                           3, 3, 3, 3, 3, 3, 3, 3,
                           4, 4, 4, 4, 4, 4, 4, 4,
                           5, 5, 5, 5, 5, 5, 5, 5,
                           6, 6, 6, 6, 6, 6, 6, 6,
                           7, 7, 7, 7, 7, 7, 7, 7,
                              8, 8, 8, 8, 8, 8    };

/*!
 * \brief Node for creating CATs (center of activity trajectories)
 *
 * This node receives bursts from the burst creator node and calculates a CAT
 * for each burst. It then publishes the CAT to the dish teleop node for
 * ARM movement.
 *
 * The CAT metric is described in this paper:
 *
 * Chao ZC, Bakkum DJ, Potter SM (2007). "Region-specific network plasticity in
 * simulated and living cortical networks: comparison of the Center of Activity
 * Trajectory (CAT) with other metrics." J. Neural Eng. 4, 294-308.
 *
 * \copyright Copyright 2013 University of Massachusetts Lowell
 * \author Jonathan Hasenzahl
 */
class CatCreator
{
public:
    CatCreator() { init(); }
    ~CatCreator();

private:
    void init();
    void getParams();
    void updateOffsets(const neuro_recv::dish_state& d);
    void callback(const burst_calc::burst::ConstPtr& b);
    void rangesCallback(const burst_calc::ranges::ConstPtr& r);
    bool caExists(const neuro_recv::dish_state& d);
    const burst_calc::ca getCa(const neuro_recv::dish_state& d);
    void initFile(const char* cat_file);
    void headerToFile(const burst_calc::burst& b);
    void toFile(int i, const neuro_recv::dish_state& d, const burst_calc::ca& c);
    void toFile(int i, const neuro_recv::dish_state& d);


    ros::NodeHandle n_;
    ros::Subscriber burst_sub_;
    ros::Subscriber ranges_sub_;
    ros::Publisher cat_pub_;
    ros::Publisher ca_pub_;
    std::ofstream cat_file_;
    std::string file_name_;
    double offsets_[60];
    double thresholds_[60];
    bool save_to_file_;
    bool is_init_;
};

#endif
