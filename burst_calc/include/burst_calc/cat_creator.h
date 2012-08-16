// =============================================================================
// Name   : cat_creator.h
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Header file for the ROS node "cat_creator".
// =============================================================================

#ifndef CAT_CREATOR_H_
#define CAT_CREATOR_H_

#include "ros/ros.h"
#include "burst_calc/burst.h"
#include "burst_calc/ranges.h"
#include "burst_calc/cat.h"
#include "burst_calc/ca.h"
#include <fstream>

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

class CatCreator
{
public:
    CatCreator() { init(); }
    ~CatCreator();

private:
    void init();
    void callback(const burst_calc::burst::ConstPtr& b);
    void rangesCallback(const burst_calc::ranges::ConstPtr& r);
    const burst_calc::ca getCa(const neuro_recv::dish_state& d);
    void initFile(const char* cat_file);
    void toFile(const burst_calc::burst& b, const burst_calc::cat& c);

    ros::NodeHandle n_;
    ros::Subscriber burst_sub_;
    ros::Subscriber ranges_sub_;
    ros::Publisher cat_pub_;
    std::ofstream cat_file_;
    double offsets_[60];
    bool save_to_file_;
    bool is_init_;
};

#endif
