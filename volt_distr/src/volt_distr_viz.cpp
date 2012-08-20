// =============================================================================
// Name   : volt_distr_viz.cpp
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Voltage distribution visualizer class for the ROS node "volt_distr". Creates
// an SVG image showing voltage tendencies of each channel.
// =============================================================================

#include "volt_distr/volt_distr_viz.h"

#define ROWS        8
#define COLS        ROWS
#define C_WIDTH     75                  // Width of each channel (px)
#define C_HEIGHT    C_WIDTH             // Height of each channel (px)
#define P_WIDTH     (C_WIDTH * COLS)    // Width of plotter window (px)
#define P_HEIGHT    (C_HEIGHT * ROWS)   // Height of plotter window (px)
#define X_START     0
#define Y_START     X_START
#define X_STEP      C_WIDTH
#define Y_STEP      X_STEP

VoltDistrViz::~VoltDistrViz()
{
    delete plotter_;
}

void VoltDistrViz::init(const std::string& file_name)
{
    is_ok_ = true;
    file_.open(file_name.c_str());

    if (!file_.is_open())
    {
        ROS_ERROR("Cannot open %s. Imaging will be disabled.",
                  file_name.c_str());
        is_ok_ = false;
        return;
    }

    // Set plotter parameters
    PlotterParams params;
    //ostringstream bitmapsize;
    //bitmapsize << P_WIDTH << 'x' << P_HEIGHT;
    //params.setplparam("BITMAPSIZE", (void*) bitmapsize.str().c_str());
    params.setplparam("BG_COLOR", (char*) "none");

    plotter_ = new SVGPlotter(cin, file_, cerr, params);

    if (plotter_->openpl() < 0)
    {
        ROS_ERROR("Cannot initialize plotter. Imaging will be disabled.");
        is_ok_ = false;
        return;
    }

    // Drawing boundaries: lower left X/Y, upper right X/Y (px)
    plotter_->space(0, 0, P_WIDTH, P_HEIGHT);
    plotter_->pencolorname("white");
    plotter_->filltype(1);
    plotter_->erase();

    // Calculate coordinates of the channels
    for (int y = ROWS - 1; y >= 0; y--)
    {
        for (int x = 0; x < COLS; x++)
        {
            if (!((x == 0 && y == 0) || (x == 0 && y == ROWS - 1) ||
                    (x == COLS - 1 && y == 0) ||
                    (x == COLS - 1 && y == ROWS - 1)))
            {
                boost::array<int, 4> coords;
                coords[0] = x * X_STEP + X_START; // x1
                coords[1] = y * Y_STEP + Y_START; // y1
                coords[2] = (x + 1) * X_STEP + X_START; // x2
                coords[3] = (y + 1) * Y_STEP + Y_START; // y2
                coords_.push_back(coords);
            }
        }
    }
}

void VoltDistrViz::draw(const boost::array<double, 60>& percents)
{
    if (!is_ok_)
        return;

    for (int i = 0; i < 60; i++)
    {
        // Draw the colored box
        if (percents[i] > 0.5)
        {
            // Channel has more than 50% negative voltages
            int red = 65535 * 2 * (percents[i] - .5);
            plotter_->fillcolor(red, 0, 0);
        }
        else
        {
            // Channel has not more than 50% negative voltages
            int blue = 65535 * 2 * (.5 - percents[i]);
            plotter_->fillcolor(0, 0, blue);
        }
        plotter_->box(coords_[i][0], coords_[i][1], coords_[i][2], coords_[i][3]);
        plotter_->endpath();

        // Draw the label over the center/center of the box
        plotter_->move((coords_[i][0] + coords_[i][2]) / 2,
                       (coords_[i][1] + coords_[i][3]) / 2);
        ostringstream volt;
        volt << fixed << setprecision(2) << (percents[i] * 100) << '%';
        plotter_->alabel('c', 'c', volt.str().c_str());
        plotter_->endpath();
    }

    plotter_->closepl();
    file_.close();
}

/*
 * Reference
 *
 * move(x,y) <-- cursor
 * box(x1,y1,x2,y2) or circle(xc,yc,r)
 * pencolor(r,g,b)
 * fillcolor(r,g,b)
 * color(r,g,b) = pen + fill
 * endpath()
 * alabel (int horiz_justify, int vert_justify, const char *s);
 */
