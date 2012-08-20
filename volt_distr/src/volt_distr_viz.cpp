// =============================================================================
// Name   : volt_distr_viz.cpp
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Voltage distribution visualizer class for the ROS node "volt_distr". Creates
// a PNG image showing voltage tendencies of each channel.
// =============================================================================

#include "volt_distr/volt_distr_viz.h"

#define ROWS        8
#define COLS        ROWS
#define C_WIDTH     25                  // Width of each channel (px)
#define C_HEIGHT    C_WIDTH             // Height of each channel (px)
#define P_WIDTH     (C_WIDTH * COLS)    // Width of plotter window (px)
#define P_HEIGHT    (C_HEIGHT * ROWS)   // Height of plotter window (px)
#define X_START     0
#define Y_START     X_START
#define X_STEP      C_WIDTH
#define Y_STEP      X_STEP

VoltDistrViz::~VoltDistrViz()
{
    if (plotter_)
    {
        plotter_->closepl();
        delete plotter_;
    }
    file_.close();
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
    ostringstream bitmapsize;
    bitmapsize << P_WIDTH << 'x' << P_HEIGHT;
    params.setplparam("BITMAPSIZE", (void*) bitmapsize.str().c_str());
    params.setplparam("BG_COLOR", (char*) "black");

    plotter_ = new PNMPlotter(cin, file_, cerr, params);

    if (plotter_->openpl() < 0)
    {
        ROS_ERROR("Cannot initialize plotter. Imaging will be disabled.");
        return;
    }

    // Lower left X/Y, upper right X/Y in px
    plotter_->space(0, 0, P_WIDTH, P_HEIGHT);

    plotter_->erase();
}
