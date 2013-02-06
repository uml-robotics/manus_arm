/*
 * burst_merger.cpp
 * Copyright 2013 University of Massachusetts Lowell
 * Author: Jonathan Hasenzahl
 */

#include "burst_calc/burst_merger.h"
#include <cstdio>

/*!
 * \brief Sets values to their initial states
 */
BurstMerger::BurstMerger()
{
    for (int i = 0; i < 60; i++)
        times_[i] = NULL;
}

/*!
 * \brief Frees up memory allocated to burst checker time pointers
 */
BurstMerger::~BurstMerger()
{
    for (int i = 0; i < 60; i++)
    {
        if (times_[i])
            delete times_[i];
    }
}

/*!
 * \brief Performs the merge operation after new bursts have been added
 *
 * First, check all stored bursts and merge any that can be. Then compare all
 * stored bursts against the times to determine if any of them can be published.
 */
void BurstMerger::update()
{
    std::vector<int> delete_indexes;

    if (list_.size() > 1)
    {
        // First, compare and merge
        for (unsigned int i = 0; i < list_.size() - 1; i++)
        {
            bool stop = false;
            for (unsigned int j = i + 1; j < list_.size() && !stop; j++)
            {
                // Determine if these bursts overlap:
                //   x.start is between y.start and y.finish OR
                //   y.start is between x.start and x.finish
                if ((list_[i].header.stamp >= list_[j].header.stamp &&
                     list_[i].header.stamp <= list_[j].end) ||
                    (list_[j].header.stamp >= list_[i].header.stamp &&
                     list_[j].header.stamp <= list_[i].end))
                {
                    printf("**  Merge   : [Sz %4d] [%6.3f - %6.3f] [Ch",
                           static_cast<int>(list_[i].dishes.size()),
                           list_[i].header.stamp.toSec(), list_[i].end.toSec());
                    for (unsigned int k = 0; k < list_[i].channels.size(); k++)
                        printf(" %d", list_[i].channels[k]);
                    printf("]\n");

                    printf("**  +       : [Sz %4d] [%6.3f - %6.3f] [Ch",
                           static_cast<int>(list_[j].dishes.size()),
                           list_[j].header.stamp.toSec(), list_[j].end.toSec());
                    for (unsigned int k = 0; k < list_[j].channels.size(); k++)
                        printf(" %d", list_[j].channels[k]);
                    printf("]\n");

                    // Merge the earlier and later bursts into the later burst
                    list_[j] = merge(list_[i], list_[j]);

                    printf("**  Result  : [Sz %4d] [%6.3f - %6.3f] [Ch",
                           static_cast<int>(list_[j].dishes.size()),
                           list_[j].header.stamp.toSec(), list_[j].end.toSec());
                    for (unsigned int k = 0; k < list_[j].channels.size(); k++)
                        printf(" %d", list_[j].channels[k]);
                    printf("]\n");

                    delete_indexes.push_back(i);
                    stop = true;
                }
            }
        }

        // Delete any elements that were merged (in reverse order so we don't
        // lose any elements unintentionally)
        if (!delete_indexes.empty())
        {
            std::vector<int>::reverse_iterator r;
            for (r = delete_indexes.rbegin(); r < delete_indexes.rend(); r++)
                list_.erase(list_.begin() + *r);
            delete_indexes.clear();
        }

        for (unsigned int i = 0; i < list_.size(); i++)
        {
            bool stop = false;
            for (int j = 0; j < 60 && !stop; j++)
            {
                if (times_[j])
                {
                    // If the start time of a channel's possible burst occurs
                    // before the merged burst's end time, then exit the
                    // loop because there is a possibility of a merge later.
                    if (*times_[j] <= list_[i].end)
                        stop = true;
                }
            }

            if (!stop)
            {
                // There is no possibility for this burst to merge with future
                // bursts on other channels, so let's get it out of here
                final_.push_back(list_[i]);
                delete_indexes.push_back(i);
            }
        }

        // Delete any elements that were finalized (in reverse order so we don't
        // lose any elements unintentionally)
        if (!delete_indexes.empty())
        {
            std::vector<int>::reverse_iterator r;
            for (r = delete_indexes.rbegin(); r < delete_indexes.rend(); r++)
                list_.erase(list_.begin() + *r);
            delete_indexes.clear();
        }
    }
}

/*!
 * \brief Updates the burst checker time pointer for a channel
 *
 * The pointer either points to the start time of that channel's possible
 * burst, or is null if there is no possible burst.
 *
 * \param index the index of the channel
 * \param t the time pointer of the channel's burst checker
 */
void BurstMerger::updateTime(int index, const ros::Time* t)
{
    if (times_[index])
        delete times_[index];

    if (t)
        // Start time of this channel's possible burst
        times_[index] = new ros::Time(*t);
    else
        // Null pointer means that there is no possible burst on this channel
        times_[index] = NULL;
}


/*!
 * \brief Merges two bursts into a single burst
 *
 * Pre-condition: the time span of the bursts must overlap in some way.
 *
 * \param b1 one of the bursts to be merged
 * \param b2 the other burst to be merged
 * \return the merged burst
 */
burst_calc::burst BurstMerger::merge(const burst_calc::burst& b1,
                                     const burst_calc::burst& b2)
{
    burst_calc::burst x = b1;
    burst_calc::burst y = b2;

    // Swap x and y if:
    //   y ends before x
    //     OR
    //   x and y end at the same time AND x starts before y
    if ((y.end < x.end) ||
       ((y.end == x.end) && (x.header.stamp < y.header.stamp)))
    {
        burst_calc::burst temp = x;
        x = y;
        y = temp;
    }

    // If x starts before y, merge x and y
    if (x.header.stamp < y.header.stamp)
    {
        ros::Time start = x.end;
        x.end = y.end;
        unsigned int index = 0;

        while (start >= y.dishes[index].header.stamp)
            index++;

        for (unsigned int i = index; i < y.dishes.size(); i++)
            x.dishes.push_back(y.dishes[i]);

        // Append y's channels to x's channels
        x.channels.insert(x.channels.end(), y.channels.begin(),
                          y.channels.end());

        return x;
    }
    // If x does not start before y, then y contains the entire burst
    else
    {
        // Append x's channels to y's channels
        y.channels.insert(y.channels.end(), x.channels.begin(),
                          x.channels.end());

        return y;
    }
}
