// =============================================================================
// Name   : time_server.h
// Author : Jonathan Hasenzahl
// Date   : 2012
//
// Header file for the ROS node "time_server".
// =============================================================================

#ifndef TIME_SERVER_H_
#define TIME_SERVER_H_

#include "ros/ros.h"
#include "time_server/time_srv.h"

class TimeServer
{
public:
    TimeServer() { init(); }

private:
    void init();
    bool service(time_server::time_srv::Request& req,
                 time_server::time_srv::Response& res);

    ros::NodeHandle n_;
    ros::ServiceServer service_;
    ros::Duration offset_;
};

#endif
