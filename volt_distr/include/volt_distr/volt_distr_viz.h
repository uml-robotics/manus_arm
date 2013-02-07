/*
 * volt_distr_viz.h
 * Copyright 2013 University of Massachusetts Lowell
 * Author: Jonathan Hasenzahl
 */

// =============================================================================
// Name   : volt_distr_viz.h
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Voltage distribution visualizer class for the ROS node "volt_distr". Creates
// an SVG image showing voltage tendencies of each channel.
// =============================================================================

#ifndef VOLT_DISTR_VIZ_H_
#define VOLT_DISTR_VIZ_H_

#include "ros/ros.h"
#include <plotter.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

/*!
 * \brief Helper class for the volt_distr node
 *
 * Voltage distribution visualizer class. Creates a SVG image showing the
 * voltage distribution of each channel.
 *
 * \copyright Copyright 2013 University of Massachusetts Lowell
 * \author Jonathan Hasenzahl
 */
class VoltDistrViz
{
public:
	/*!
	 * \param file_name the file path for saving the image
	 */
    VoltDistrViz(const std::string& file_name) { init(file_name); }
    ~VoltDistrViz();

    void draw(const boost::array<double, 60>& percents);

private:
    void init(const std::string& file_name);

    SVGPlotter* plotter_;
    std::ofstream file_;
    std::vector< boost::array<int, 4> > coords_;

    bool is_ok_;
};

#endif
