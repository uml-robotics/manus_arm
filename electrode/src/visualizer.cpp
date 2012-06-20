// =============================================================================
// Name   : visualizer.cpp
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Implements the ROS node "visualizer". This is a test node used to inspect the
// "spike_state" messages published by the "spike_detector" node.
// =============================================================================
// !!! BROKEN AS OF 6/15/12 DUE TO CHANGES TO SPIKE DETECTOR NODE !!!

#include "electrode/visualizer.h"
#include <ctime>

#define SPIKE '@'
#define NO_SPIKE '.'
#define SLEEP_TIME 1

Visualizer::Visualizer()
{
    //spike_state_sub_ = n_.subscribe("spike_states", 1000, &Visualizer::callback, this);
    for (int i = 0; i < 8; i++)
        for (int j = 0; j < 8; j++)
            grid_[i][j] = false;
}

void Visualizer::init()
{
    ROS_INFO("Visualizer running...");
    ros::spin();
}

coord Visualizer::centerOfActivity()
{
    double x_sum = 0;
    double y_sum = 0;
    int weight = 0;
    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            if (grid_[i][j])
            {
                x_sum += j;
                y_sum += i;
                weight++;
            }
        }
    }

    coord coa;
    if (weight > 0)
    {
        coa.x = x_sum / weight;
        coa.y = y_sum / weight;
        coa.exists = true;
    }
    else
    {
        coa.exists = false;
    }
    return coa;
}

void Visualizer::display()
{
    coord coa = centerOfActivity();
    if (coa.exists)
        printf("\nCenter of Activity: X[%f] Y[%f]\n\n", coa.x, coa.y);
    else
        printf("\nNo center of activity in this snapshot.\n\n");
    printf("   0  1  2  3  4  5  6  7\n\n");
    for (int i = 0; i < 8; i++)
    {
        printf("%d", i);
        for (int j = 0; j < 8; j++)
            printf("  %c", grid_[i][j] ? '@' : '.');
        printf("\n\n");
    }
    sleep(SLEEP_TIME);
}

void Visualizer::sleep(int seconds)
{
    time_t start = time(NULL);
    while (start + seconds > time(NULL));
}

/*void Visualizer::callback(const electrode::spike_state::ConstPtr& dish)
{
    if (ros::ok())
    {
        // Populate row 0 (exclude columns 0 and 7)
        for (int i = 0; i < 6; i++)
            grid_[0][i+1] = dish->spikes[i];

        // Populate rows 1-6 (all columns)
        int row = 1;
        int col = 0;
        for (int i = 6; i < 54; i++)
        {
            grid_[row][col++] = dish->spikes[i];
            if ((i - 5) % 8 == 0) row++;
        }

        // Populate row 7 (exclude columns 0 and 7)
        for (int i = 54; i < 60; i++)
            grid_[7][i-53] = dish->spikes[i];

        display();
    }
}*/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visualizer");
    Visualizer visualizer;
    visualizer.init();
    return 0;
}
