// =============================================================================
// Name   : volt_distr_viz.h
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Voltage distribution visualizer class for the ROS node "volt_distr". Creates
// a PNG image showing voltage tendencies of each channel.
// =============================================================================

#ifndef VOLT_DISTR_VIZ_H_
#define VOLT_DISTR_VIZ_H_

#include "ros/ros.h"
#include <plotter.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

class VoltDistrViz
{
public:
    VoltDistrViz(const std::string& file_name) { init(file_name); }
    ~VoltDistrViz();

    void draw(const boost::array<double, 60>& percents);

private:
    void init(const std::string& file_name);

    PNMPlotter* plotter_;
    std::ofstream file_;
    std::vector< boost::array<int, 4> > coords_;

    bool is_ok_;
};

#endif
