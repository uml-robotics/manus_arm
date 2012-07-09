/*
 * Visualizer.h
 *
 *  Created on: Mar 29, 2012
 *      Author: ams
 */

#ifndef _DISH_VISUALIZER_H
#define _DISH_VISUALIZER_H

#include <plotter.h>
#include <sstream>
#include <vector>
#include <stdint.h>
#include "ros/ros.h"
#include "boost/thread.hpp"
#include "boost/range.hpp"

class DishVisualizer {
private:
	XPlotter *plotter;
	vector< vector<int> > centers;
	vector<double> data;
	int intMap(double input, double min_in, double max_in, int min_out, int max_out);
	boost::mutex dataUpdate;
	double baselines[60];
	double min_volts[60];
	double max_volts[60];
public:
	DishVisualizer();
	virtual ~DishVisualizer();
	int init();
	bool isInit;
	void update(int channel, double newValue);
	void redraw();
	void setVoltRanges(const boost::array<double, 60>& b,
	                   const boost::array<double, 60>& min,
	                   const boost::array<double, 60>& max);
};

#endif /* _DISH_VISUALIZER_H */
