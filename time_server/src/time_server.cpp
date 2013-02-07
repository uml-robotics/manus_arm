/*
 * time_server.cpp
 * Copyright 2013 University of Massachusetts Lowell
 * Author: Jonathan Hasenzahl
 */

#include "time_server/time_server.h"

/*!
 * \brief Initializes the node
 *
 * Advertises the service and starts the spin loop.
 */
void TimeServer::init()
{
    service_ = n_.advertiseService("time_service", &TimeServer::service, this);
    ros::spin();
}

/*!
 * \brief Callback for time service request
 *
 * This method is called automatically when the node receives a service request.
 * The node responds with the current clock time and the difference between
 * the request time and the actual time.
 *
 * If this is the first request received, the node starts its clock (from 0).
 *
 * \param req the client request
 * \param res the server response
 * \return true
 */
bool TimeServer::service(time_server::time_srv::Request& req,
                         time_server::time_srv::Response& res)
{
    // To start the clock, we wait for the first service call. The difference
    // between the actual start of dish generation in a receiver node and the
    // start of the clock here is only a few milliseconds.
    static bool run_once = true;
    if (run_once)
    {
        offset_ = ros::Time::now() - ros::Time(0);
        ROS_INFO("Time server seeded successfully");
        run_once = false;
    }

    // Calculate the responses
    res.actual = ros::Time::now() - offset_;
    res.delta = req.target - res.actual;

    printf("Query: %.6fs Actual %.6fs Delta %.6fs\n", req.target.toSec(),
           res.actual.toSec(), res.delta.toSec());

    return true;
}

/*!
 * \brief Creates an instance of the node
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "time_server");
    TimeServer time_server;
    return 0;
}
