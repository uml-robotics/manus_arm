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

    ros::spin();
}

bool TimeServer::service(time_server::time_srv::Request& req,
                         time_server::time_srv::Response& res)
{
    static bool run_once = true;
    if (run_once)
    {
        offset_ = req.target - ros::Time(0);
        run_once = false;
    }

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
