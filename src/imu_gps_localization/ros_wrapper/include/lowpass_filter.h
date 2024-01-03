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

class LowPassFilter
{
public:
    LowPassFilter(double cutoff_frequency, double sampling_frequency)
    {
        double alpha = 0.03;
        previous_filtered_value_ = Eigen::Vector3d::Zero();
        alpha_ = alpha;
    }

    Eigen::Vector3d update(const Eigen::Vector3d &current_value)
    {
        filtered_value_ = alpha_ * current_value + (1.0 - alpha_) * previous_filtered_value_;
        previous_filtered_value_ = filtered_value_;
        return filtered_value_;
    }

private:
    double alpha_;
    Eigen::Vector3d previous_filtered_value_;
    Eigen::Vector3d filtered_value_;
};