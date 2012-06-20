// =============================================================================
// Name   : ca_calculator.h
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// CA calculator class for the ROS node "cat_creator". Calculates CA (center of
// activity) for individual dishes in a burst sequence.
// =============================================================================

#ifndef CA_CALCULATOR_H_
#define CA_CALCULATOR_H_

#include "electrode/dish_state.h"
#include "electrode/ca.h"

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

class CaCalculator
{
public:
    static const electrode::ca getCa(const electrode::dish_state& d)
    {
        // Center of activity = summation(position*activity) / total activity
        double x_sum = 0.0;
        double y_sum = 0.0;
        double activity = 0.0;

        // Calculate offset to account for negative readings messing with the
        // algorithm
        double offset = d.samples[0];
        for (int i = 1; i < 60; i++)
        {
            if (d.samples[i] < offset)
                offset = d.samples[i];
        }

        // If offset is positive, set it to zero as it is not needed
        if (offset > 0.0)
            offset = 0.0;

        for (int i = 0; i < 60; i++)
        {
            //if (d.samples[i] != 9.99)
            //{
                x_sum += (d.samples[i] - offset) * X_COORD_[i];
                y_sum += (d.samples[i] - offset) * Y_COORD_[i];
                activity += d.samples[i] - offset;
            //}
        }

        electrode::ca ca;
        ca.header.stamp.sec = d.header.stamp.sec;
        ca.header.stamp.nsec = d.header.stamp.nsec;
        ca.x = x_sum / activity;
        ca.y = y_sum / activity;
        //printf("ca.x = %f / %f = %f\n", x_sum, activity, ca.x);
        //printf("ca.y = %f / %f = %f\n", y_sum, activity, ca.y);

        return ca;
    }
};

#endif
