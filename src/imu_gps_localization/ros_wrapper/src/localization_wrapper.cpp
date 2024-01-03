#include "localization_wrapper.h"

#include "lowpass_filter.h"

#include <nav_msgs/Odometry.h>

#include <iomanip>

#include <glog/logging.h>

#include "imu_gps_localizer/base_type.h"

LocalizationWrapper::LocalizationWrapper(ros::NodeHandle &nh)
{ // 2, LocalizationWrapper 构造函数；
    // Load configs.
    double acc_noise, gyro_noise, acc_bias_noise, gyro_bias_noise; // 2.1.1 读入 ESKF 需要用到的噪声参数；
    nh.param("acc_noise", acc_noise, 0.2);                         // a_n;  // noise 一般是指白噪声；
    nh.param("gyro_noise", gyro_noise, 1e-4);                      // w_n;
    nh.param("acc_bias_noise", acc_bias_noise, 1e-6);              // a_b;  // bias 是随时间缓慢变化的噪声；
    nh.param("gyro_bias_noise", gyro_bias_noise, 1e-8);            // w_b;

    double x, y, z;
    nh.param("I_p_Gps_x", x, 0.);
    nh.param("I_p_Gps_y", y, 0.);
    nh.param("I_p_Gps_z", z, 0.);
    const Eigen::Vector3d I_p_Gps(x, y, z); // 2.1.2 IMU坐标系下的GPS坐标；同理，G_p_I：全局坐标系下IMU的位置；G_R_I：全局坐标系下IMU的姿态；G_v_I：全局坐标系下IMU的速度；

    std::string log_folder = "/home/cxy/slam/src/imu_gps_localization/Log/";
    ros::param::get("log_folder", log_folder); // 2.1.3 设置日志保存路径；

    // Log.
    file_state_.open(log_folder + "/state.csv");
    file_gps_.open(log_folder + "/gps.csv");

    // Initialization imu gps localizer.
    imu_gps_localizer_ptr_ =
        std::make_unique<ImuGpsLocalization::ImuGpsLocalizer>(acc_noise, gyro_noise,
                                                              acc_bias_noise, gyro_bias_noise,
                                                              I_p_Gps); // 2.2 初始化 ImuGpsLocalizer 对象，用于ESKF的预测（由IMU负责）和更新（由GPS负责）；

    // Subscribe topics.
    imu_sub_ = nh.subscribe("/airsim_node/drone_1/imu/imu", 10, &LocalizationWrapper::ImuCallback, this);               // 2.3.1 接收imu消息，转成ImuGpsLocalization::ImuData结构体类型，并用 imu_gps_localizer 处理IMU数据，并保存IMU数据日志；
    gps_position_sub_ = nh.subscribe("/airsim_node/drone_1/pose", 10, &LocalizationWrapper::GpsPositionCallback, this); // 2.3.2 接GPS数据并转成ImuGpsLocalization::GpsPositionData类型，用 imu_gps_localizer 处理GPS数据，并保存GPS数据日志；
    pose_sub_ = nh.subscribe("/airsim_node/drone_1/debug/pose_gt", 10, &LocalizationWrapper::PositionCallback, this);   // 2.3.2 接GPS数据并转成ImuGpsLocalization::GpsPositionData类型，用 imu_gps_localizer 处理GPS数据，并保存GPS数据日志；")

    state_pub_ = nh.advertise<nav_msgs::Path>("fused_path", 10);  // 发布位姿路径；
    pose_pub_ = nh.advertise<nav_msgs::Path>("gt_path", 10);      // 发布位姿路径；
    gps_pub_ = nh.advertise<nav_msgs::Path>("gps_path", 10);      // 发布位姿路径；
    odometry_ = nh.advertise<nav_msgs::Odometry>("/odometry", 10); // 发布里程计；
}

LocalizationWrapper::~LocalizationWrapper()
{
    file_state_.close();
    file_gps_.close();
}

void LocalizationWrapper::ImuCallback(const sensor_msgs::ImuConstPtr &imu_msg_ptr)
{ // 2.3.1
    ImuGpsLocalization::ImuDataPtr imu_data_ptr = std::make_shared<ImuGpsLocalization::ImuData>();
    imu_data_ptr->timestamp = imu_msg_ptr->header.stamp.toSec();
    imu_data_ptr->acc << imu_msg_ptr->linear_acceleration.x,
        imu_msg_ptr->linear_acceleration.y,
        imu_msg_ptr->linear_acceleration.z;
    imu_data_ptr->gyro << imu_msg_ptr->angular_velocity.x,
        imu_msg_ptr->angular_velocity.y,
        imu_msg_ptr->angular_velocity.z;

    ImuGpsLocalization::State fused_state;
    const bool ok = imu_gps_localizer_ptr_->ProcessImuData(imu_data_ptr, &fused_state); // 2.3.1 处理IMU数据，得到融合后的状态 fused_state;；==> 3, imu_gps_localizer.cpp, ProcessImuData
    if (!ok)
    {
        return;
    }

    // Publish fused state.
    fused_state.lla_afterfilter = low_pass_filter.update(fused_state.G_p_I); // 对位置进行低通滤波
    ConvertStateToRosTopic(fused_state);                                     // 将状态结构体类数据型转换成ros对应的消息类型
    ConvertStateToOdometry(fused_state);                                     // 将状态结构体类数据型转换成ros对应的消息类型
    state_pub_.publish(ros_path_);                                           // 发布消息；

    // Log fused state.
    LogState(fused_state); // 保存 fused_state 轨迹数据日志；
}

void LocalizationWrapper::GpsPositionCallback(const geometry_msgs::PoseStampedConstPtr &gps_msg_ptr)
{ // 2.3.2
    // Check the gps_status.
    // if (gps_msg_ptr->status.status != 2)
    // { // 在 fix 消息中，GPS数据的 status 表示卫星定位状态信息；（-1，无法确定位置；0，未能精准定位；1，with satellite-based augmentation；2，with ground-based augmentation）
    //     LOG(WARNING) << "[GpsCallBack]: Bad gps message!";
    //     return;
    // }

    ImuGpsLocalization::GpsPositionDataPtr gps_data_ptr = std::make_shared<ImuGpsLocalization::GpsPositionData>();
    gps_data_ptr->timestamp = gps_msg_ptr->header.stamp.toSec();
    gps_data_ptr->lla << gps_msg_ptr->pose.position.x,
        gps_msg_ptr->pose.position.y,
        gps_msg_ptr->pose.position.z;
    gps_data_ptr->cov = Eigen::Matrix3d::Identity() * 400.0;

    imu_gps_localizer_ptr_->ProcessGpsPositionData(gps_data_ptr); // 2.3.2 处理GPS数据，用于状态更新；==> 3, imu_gps_localizer.cpp, ProcessGpsPositionData

    LogGps(gps_data_ptr); // 保存GPS数据日志；
    gps_path_.header.frame_id = "world";
    geometry_msgs::PoseStamped pose;
    pose.header = ros_path_.header;
    pose.header.stamp = gps_msg_ptr->header.stamp;

    pose.pose.position.x = gps_msg_ptr->pose.position.x;
    pose.pose.position.y = gps_msg_ptr->pose.position.y;
    pose.pose.position.z = gps_msg_ptr->pose.position.z;

    pose.pose.orientation.x = gps_msg_ptr->pose.orientation.x;
    pose.pose.orientation.y = gps_msg_ptr->pose.orientation.y;
    pose.pose.orientation.z = gps_msg_ptr->pose.orientation.z;
    pose.pose.orientation.w = gps_msg_ptr->pose.orientation.w;

    gps_path_.poses.push_back(pose);
    gps_pub_.publish(gps_path_);
}

void LocalizationWrapper::PositionCallback(const geometry_msgs::PoseStampedConstPtr &gt_msg_ptr)
{
    gt_path_.header.frame_id = "world";
    geometry_msgs::PoseStamped pose;
    pose.header = ros_path_.header;
    pose.header.stamp = gt_msg_ptr->header.stamp;

    pose.pose.position.x = gt_msg_ptr->pose.position.x;
    pose.pose.position.y = -gt_msg_ptr->pose.position.y;
    pose.pose.position.z = -gt_msg_ptr->pose.position.z;

    pose.pose.orientation.x = gt_msg_ptr->pose.orientation.x;
    pose.pose.orientation.y = gt_msg_ptr->pose.orientation.y;
    pose.pose.orientation.z = gt_msg_ptr->pose.orientation.z;
    pose.pose.orientation.w = gt_msg_ptr->pose.orientation.w;

    gt_path_.poses.push_back(pose);
    pose_pub_.publish(gt_path_);
}

void LocalizationWrapper::LogState(const ImuGpsLocalization::State &state)
{
    const Eigen::Quaterniond G_q_I(state.G_R_I);
    file_state_ << std::fixed << std::setprecision(15)
                << state.timestamp << ","
                << state.lla[0] << "," << state.lla[1] << "," << state.lla[2] << ","
                << state.G_p_I[0] << "," << state.G_p_I[1] << "," << -state.G_p_I[2] << ","
                << state.G_v_I[0] << "," << state.G_v_I[1] << "," << state.G_v_I[2] << ","
                << G_q_I.x() << "," << G_q_I.y() << "," << G_q_I.z() << "," << G_q_I.w() << ","
                << state.acc_bias[0] << "," << state.acc_bias[1] << "," << state.acc_bias[2] << ","
                << state.gyro_bias[0] << "," << state.gyro_bias[1] << "," << state.gyro_bias[2] << "\n";
}

void LocalizationWrapper::LogGps(const ImuGpsLocalization::GpsPositionDataPtr gps_data)
{ // 保存GPS数据的时间戳，经纬度，高程值到日志文件；
    file_gps_ << std::fixed << std::setprecision(15)
              << gps_data->timestamp << ","
              << gps_data->lla[0] << "," << gps_data->lla[1] << "," << gps_data->lla[2] << "\n";
}

void LocalizationWrapper::ConvertStateToRosTopic(const ImuGpsLocalization::State &state)
{ // 提取 state 中的当前位姿并转成geometry_msgs::PoseStamped类型发布出去；
    ros_path_.header.frame_id = "world";
    ros_path_.header.stamp = ros::Time::now();

    geometry_msgs::PoseStamped pose;
    pose.header = ros_path_.header;

    pose.pose.position.x = state.lla_afterfilter[0];
    pose.pose.position.y = -state.lla_afterfilter[1];
    pose.pose.position.z = -state.lla_afterfilter[2];

    const Eigen::Quaterniond G_q_I(state.G_R_I);
    pose.pose.orientation.x = G_q_I.x();
    pose.pose.orientation.y = G_q_I.y();
    pose.pose.orientation.z = G_q_I.z();
    pose.pose.orientation.w = G_q_I.w();

    ros_path_.poses.push_back(pose);
}

void LocalizationWrapper::ConvertStateToOdometry(const ImuGpsLocalization::State &state)
{
    odometry_msg_.header.frame_id = "world";
    odometry_msg_.header.stamp = ros::Time::now();

    odometry_msg_.pose.pose.position.x = state.lla_afterfilter[0];
    odometry_msg_.pose.pose.position.y = state.lla_afterfilter[1];
    odometry_msg_.pose.pose.position.z = state.lla_afterfilter[2];

    Eigen::Quaterniond odom_q;
    odom_q = Eigen::Quaterniond(state.G_R_I);
    odom_q.normalize();

    odometry_msg_.pose.pose.orientation.x = odom_q.x();
    odometry_msg_.pose.pose.orientation.y = odom_q.y();
    odometry_msg_.pose.pose.orientation.z = odom_q.z();
    odometry_msg_.pose.pose.orientation.w = odom_q.w();

    odometry_msg_.child_frame_id = "base_link";
    odometry_msg_.twist.twist.linear.x = state.G_v_I[0];
    odometry_msg_.twist.twist.linear.y = state.G_v_I[1];
    odometry_msg_.twist.twist.linear.z = state.G_v_I[2];

    odometry_.publish(odometry_msg_);
}
