/*
 * time_server.h
 * Copyright 2013 University of Massachusetts Lowell
 * Author: Jonathan Hasenzahl
 */

#ifndef TIME_SERVER_H_
#define TIME_SERVER_H_

#include "ros/ros.h"
#include "time_server/time_srv.h"

/*!
 * \brief Node that provides a time service to keep other nodes in sync.
 *
 * The clock starts at 0, and begins the first time the service is called.
 *
 * \copyright Copyright 2013 University of Massachusetts Lowell
 * \author Jonathan Hasenzahl
 */
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
