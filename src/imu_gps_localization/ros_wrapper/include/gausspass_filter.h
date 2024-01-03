#pragma once

#include <fstream>
#include <memory>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <Eigen/Dense>

class GaussPass_filter
{
private:
    /* data */
public:
    GaussPass_filter(/* args */)
    {
    }
};
