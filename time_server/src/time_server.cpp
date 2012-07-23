// =============================================================================
// Name   : time_server.cpp
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Implements the ROS node "time_server".
// =============================================================================

#include "time_server/time_server.h"

void TimeServer::init()
{
    service_ = n_.advertiseService("time_service", &TimeServer::service, this);
    ROS_INFO("Time server running...");
    ros::spin();
}

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
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "time_server");
    TimeServer time_server;
    time_server.init();
    return 0;
}
