/*
 * DishVisualizer.h
 * Copyright 2013 University of Massachusetts Lowell
 * Authors: Abraham Shultz, Jonathan Hasenzahl
 */

#ifndef _DISH_VISUALIZER_H
#define _DISH_VISUALIZER_H

#include <plotter.h>
#include <sstream>
#include <vector>
#include <stdint.h>
#include "ros/ros.h"
#include "burst_calc/ca.h"
#include "boost/thread.hpp"
#include "boost/range.hpp"

#define RED_BLUE_SEPARATED 0
#define RED_BLUE_MIX 1
#define RED_GREEN_BLUE 2
#define BLUE_ONLY 3

#define MAX_COLOR 65535

/*!
 * \brief Helper class for the dish_viz node
 *
 * Creates a visualizer on screen so the user can see the spiking activity
 * which is causing ARM movement.
 *
 * \copyright Copyright 2013 University of Massachusetts Lowell
 * \author Abraham Shultz
 * \author Jonathan Hasenzahl
 */
class DishVisualizer {
private:
	XPlotter *plotter;
	vector< vector<int> > centers;
	vector<double> data;
	burst_calc::ca ca;
	bool plot_ca;
	int intMap(double input, double min_in, double max_in, int min_out, int max_out);
	boost::mutex dataUpdate;
	double baselines[60];
	double thresholds[60];
	double min_volts[60];
	double max_volts[60];
	int color_mode;

public:
	DishVisualizer();
	virtual ~DishVisualizer();
	int init(int mode);
	bool isInit;
	void update(int channel, double newValue);
	void updateCa(const burst_calc::ca& c);
	void redraw();
	void setVoltRanges(const boost::array<double, 60>& b,
	                   const boost::array<double, 60>& t,
	                   const boost::array<double, 60>& min,
	                   const boost::array<double, 60>& max);

	double getMinVolt(int index) { return min_volts[index]; }
	double getMaxVolt(int index) { return max_volts[index]; }
	void setMinVolt(int index, double volt) { min_volts[index] = volt; }
	void setMaxVolt(int index, double volt) { max_volts[index] = volt; }
};

#endif /* _DISH_VISUALIZER_H */
