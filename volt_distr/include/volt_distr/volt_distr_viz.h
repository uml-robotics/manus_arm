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

class VoltDistrViz
{
public:
    VoltDistrViz(const std::string& file_name) { init(file_name); }
    ~VoltDistrViz();

    bool ok() { return is_ok_; }

private:
    void init(const std::string& file_name);

    PNMPlotter* plotter_;
    std::ofstream file_;

    bool is_ok_;
};

#endif
