#pragma once

#include <fstream>
#include <memory>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include "lowpass_filter.h"

#include "imu_gps_localizer/imu_gps_localizer.h"

class LocalizationWrapper
{
public:
    LocalizationWrapper(ros::NodeHandle &nh);
    ~LocalizationWrapper();

    void ImuCallback(const sensor_msgs::ImuConstPtr &imu_msg_ptr);

    // void GpsPositionCallback(const sensor_msgs::NavSatFixConstPtr &gps_msg_ptr);
    void GpsPositionCallback(const geometry_msgs::PoseStampedConstPtr &gps_msg_ptr);
    void PositionCallback(const geometry_msgs::PoseStampedConstPtr &gt_msg_ptr);

private:
    void LogState(const ImuGpsLocalization::State &state);
    void LogGps(const ImuGpsLocalization::GpsPositionDataPtr gps_data);

    void ConvertStateToRosTopic(const ImuGpsLocalization::State &state);

    void ConvertStateToOdometry(const ImuGpsLocalization::State &state);

    ros::Subscriber imu_sub_;
    ros::Subscriber gps_position_sub_;
    ros::Subscriber pose_sub_;
    ros::Publisher state_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher gps_pub_;
    ros::Publisher odometry_;

    std::ofstream file_state_;
    std::ofstream file_gps_;

    nav_msgs::Path ros_path_;
    nav_msgs::Path gt_path_;
    nav_msgs::Path gps_path_;
    nav_msgs::Odometry odometry_msg_;

    std::unique_ptr<ImuGpsLocalization::ImuGpsLocalizer> imu_gps_localizer_ptr_;
    LowPassFilter low_pass_filter{10, 100};
};