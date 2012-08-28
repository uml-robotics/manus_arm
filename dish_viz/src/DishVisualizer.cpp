/*
 * DishVisualizer.cpp
 *
 *  Created on: Jun 4, 2012
 *      Author: ams
 */

#include "dish_viz/DishVisualizer.h"

#define RADIUS 20
#define P_WIDTH ((7 * RADIUS) + (8 * (2 * RADIUS)) + (2 * RADIUS))
#define P_HEIGHT P_WIDTH
#define X_STEP (3*RADIUS)
#define Y_STEP X_STEP
#define START_X (2*RADIUS)
#define START_Y START_X
#define ROWS 8
#define COLS ROWS

DishVisualizer::DishVisualizer() {
	isInit = FALSE;
	data.assign(60, 0);
	for (int i = 0; i < 60; i++)
	{
	    baselines[i] = 0.0;
	    thresholds[i] = 0.0;
	    min_volts[i] = 0.0;
	    max_volts[i] = 0.0;
	}
}

DishVisualizer::~DishVisualizer() {
	isInit = FALSE;
}

int DishVisualizer::init(int mode) {
	isInit = FALSE;

	// Set color mode
    if (mode >= 0 && mode <= 3)
    {
        color_mode = mode;
        ROS_INFO("Visualizer color mode set to %d", color_mode);
    }
    else
    {
        color_mode = RED_BLUE_SEPARATED;
        ROS_WARN("Color mode (%d) is out of range [0...3]", mode);
    }

	/* set Plotter parameters */
	PlotterParams plotter_params;
	//convert width and height to config string
	ostringstream oss;
	oss << P_WIDTH << "x" << P_HEIGHT;
	plotter_params.setplparam("BITMAPSIZE", (void*) oss.str().c_str());
	//plotter_params.setplparam("VANISH_ON_DELETE", (char *)"yes");
	plotter_params.setplparam("USE_DOUBLE_BUFFERING", (char *) "yes");

	/* create an X Plotter with the specified parameters */
	plotter = new XPlotter(stdin, stdout, stderr, plotter_params);

	if (plotter->openpl() < 0) /* open Plotter */
	{
		ROS_FATAL("Couldn't open Plotter");
		return 1;
	}

	plotter->space(0, 0, P_WIDTH, P_HEIGHT); //coordinates in pixels, not (0,0)-(1,1)
	plotter->flinewidth(0.01); //line thickness
	plotter->filltype(1); //Fill objects
	plotter->bgcolor(0, 0, 0); //Black, colors are RGB, 16 bits/channel
	plotter->erase();

	// Draw lines to separate grid into quadrants
	plotter->pencolorname("white");
	plotter->line(P_WIDTH / 2, 0, P_WIDTH / 2, P_HEIGHT);
	plotter->line(0, P_HEIGHT / 2, P_WIDTH, P_HEIGHT / 2);
	plotter->endpath();

	for (int row = ROWS - 1; row >= 0; row--) {
		for (int col = 0; col < COLS; col++) {
			//Calculate the locations of the circles
			if (!((row == 0 && col == 0) || (row == ROWS - 1 && col == COLS - 1)
					|| (row == 0 && col == COLS - 1)
					|| (row == ROWS - 1 && col == 0))) {

				//int index = (COLS * row) + col;
				int xPos = START_X + (col * X_STEP);
				int yPos = START_Y + (row * Y_STEP);
				//ROS_INFO("%d, %d, %d", index, xPos, yPos);
				//Stash them in the array
				vector<int> tmp;
				tmp.push_back(xPos);
				tmp.push_back(yPos);
				centers.push_back(tmp);
				plotter->color(65535, 0, 0); //Set pen and fill color
				plotter->circle(xPos, yPos, RADIUS); //Draw a circle at the location
			}
		}
	}
	plotter->endpath();
	plotter->erase();

	//Start up updating thread
	boost::thread visualUpdate(&DishVisualizer::redraw, this);

	isInit = TRUE;
	return 0;

}

int DishVisualizer::intMap(double input, double min_in, double max_in,
		int min_out, int max_out) {
	int retVal = 0;
	//constrain to the range
	if (input > max_in) {
		input = max_in;
	} else if (input < min_in) {
		input = min_in;
	}

	//map the constrained value into the range
	retVal = (int) ((input - (float) min_in) * (max_out - min_out)
			/ ((float) max_in - (float) min_in) + min_out);
	return retVal;
}

void DishVisualizer::redraw() {
	while (isInit) {
	    // Draw lines to separate grid into quadrants
        plotter->pencolorname("white");
        plotter->line(P_WIDTH / 2, 0, P_WIDTH / 2, P_HEIGHT);
        plotter->line(0, P_HEIGHT / 2, P_WIDTH, P_HEIGHT / 2);
        plotter->endpath();

        for (uint showChan = 0; showChan < data.size(); showChan++) {
            uint16_t red = 0;
            uint16_t green = 0;
            uint16_t blue = 0;

            switch (color_mode)
            {
                // Red/blue separated:
                //     Channels at/under threshold are a red gradient
                //     Channels above threshold are solid blue
                case RED_BLUE_SEPARATED:
                    if (data[showChan] <= thresholds[showChan])
                        red = intMap(data[showChan], min_volts[showChan],
                                     thresholds[showChan], MAX_COLOR / 2, 0);
                    else
                        blue = MAX_COLOR;
                    break;

                // Red/blue mix:
                //     Channels have a mix of red and blue. Mostly red
                //     indicates a low value and mostly blue indicates a
                //     high value.
                case RED_BLUE_MIX:
                    red = intMap(data[showChan], min_volts[showChan],
                                 max_volts[showChan], MAX_COLOR, 0);
                    blue = intMap(data[showChan], min_volts[showChan],
                                 max_volts[showChan], 0, MAX_COLOR);
                    break;

                // Red/green/blue:
                //     Channels at/under baseline are a red gradient
                //     Channels above baseline and at/under threshold are
                //       a green gradient
                //     Channels above threshold are solid blue
                case RED_GREEN_BLUE:
                    if (data[showChan] <= baselines[showChan])
                        red = intMap(data[showChan], min_volts[showChan],
                                     thresholds[showChan], MAX_COLOR, 0);
                    else if (data[showChan] <= thresholds[showChan])
                        green = intMap(data[showChan], min_volts[showChan],
                                       thresholds[showChan], MAX_COLOR, 0);
                    else
                        blue = MAX_COLOR;
                    break;

                // Blue only:
                //     Channels at/under threshold are black
                //     Channels above threshold are solid blue
                case BLUE_ONLY:
                    if (data[showChan] > thresholds[showChan])
                        blue = MAX_COLOR;
                    break;

                default:
                    ROS_ERROR("Color mode (%d) is out of range [0...3]",
                              color_mode);
                    color_mode = RED_BLUE_SEPARATED;
                    break;
            }

            plotter->color(red, green, blue);

            //Get the center coordinates and draw the circle
            int xPos = centers[showChan][0];
            int yPos = centers[showChan][1];

            //ROS_INFO("%d = %f: %d, %d, %d at (%d, %d)", showChan, data[showChan], red, green, blue, xPos, yPos);
            plotter->circle(xPos, yPos, RADIUS);
		}

		plotter->erase();
		//sleep for a 60th of a second
		boost::this_thread::sleep(boost::posix_time::millisec(16));
	}
}

void DishVisualizer::update(int channel, double newValue) {
	if (isInit) {
		boost::mutex::scoped_lock lock(dataUpdate);
		data[channel]= newValue;
		//ROS_INFO("Channel %d (of %d) = %f", channel, data.size(), newValue);
	} else {
		ROS_WARN("Visualizer updated while not initialized");
	}
}

void DishVisualizer::setVoltRanges(const boost::array<double, 60>& b,
                                   const boost::array<double, 60>& t,
                                   const boost::array<double, 60>& min,
                                   const boost::array<double, 60>& max)
{
    for (int i = 0; i < 60; i++)
    {
        baselines[i] = b[i];
        thresholds[i] = t[i];
        min_volts[i] = min[i];
        max_volts[i] = max[i];
    }
}
