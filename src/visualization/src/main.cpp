#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <dji_msgs/Loop.h>
#include <dji_msgs/Trajectory.h>
#include <nav_msgs/Odometry.h>

ros::Publisher v_loop_pub;
ros::Publisher v_trajectory_pub;
ros::Publisher uav_fly_pub;

// 圆环可视化
void loop_pos_cb(const dji_msgs::Loop::ConstPtr& msg) {
  visualization_msgs::Marker v_loop_msg;
  v_loop_msg.header.frame_id = "world";
  v_loop_msg.header.stamp = ros::Time::now();

  v_loop_msg.type = visualization_msgs::Marker::LINE_STRIP;
  v_loop_msg.action = visualization_msgs::Marker::MODIFY;
  v_loop_msg.id = msg->loop_id;
  
  if (v_loop_msg.id > 4 && v_loop_msg.id <= 11)
  {
    v_loop_msg.scale.x = 0.1;
    v_loop_msg.color.r = 1.0;
    v_loop_msg.color.g = 1.0;
    v_loop_msg.color.b = 0.0;
    v_loop_msg.color.a = 1.0; 
  }
  else
  {
    v_loop_msg.scale.x = 0.1;
    v_loop_msg.color.r = 1.0;
    v_loop_msg.color.g = 0.0;
    v_loop_msg.color.b = 0.0;
    v_loop_msg.color.a = 1.0; 
  }
  // 计算并设置圆环上的点
  for (double angle = 0; angle <= 2 * M_PI; angle += M_PI / 180.0) {
    geometry_msgs::Point p;
    p.x = msg->loop_pos.x + 0.7 * cos(angle);  // 0.7作为圆环的半径
    p.y = -msg->loop_pos.y + 0.7 * sin(angle); 
    p.z = -msg->loop_pos.z;
    v_loop_msg.points.push_back(p);
  }
  v_loop_pub.publish(v_loop_msg);

}

// 轨迹可视化
void trajectory_cb(const dji_msgs::Trajectory::ConstPtr& msg) {
  nav_msgs::Path v_trajectory_msg;
  v_trajectory_msg.header.frame_id = "world";
  v_trajectory_msg.header.stamp = ros::Time::now();

  for (int i = 0; i < msg->pos.size(); i++) {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = msg->pos[i].x;
    pose.pose.position.y = -msg->pos[i].y;
    pose.pose.position.z = -msg->pos[i].z;
    double yaw = msg->yaw[i];
    pose.pose.orientation.w = cos(yaw/2);
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = sin(yaw/2);
    v_trajectory_msg.poses.push_back(pose);
  }
  v_trajectory_pub.publish(v_trajectory_msg);
}

std::vector<geometry_msgs::PoseStamped> poses;
void fly_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  nav_msgs::Path uav_fly_msg;
  uav_fly_msg.header.frame_id = "world";
  uav_fly_msg.header.stamp = ros::Time::now();
  
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = msg->pose.position.x;
  pose.pose.position.y = -msg->pose.position.y;
  pose.pose.position.z = -msg->pose.position.z;
  pose.pose.orientation.w = msg->pose.orientation.w;
  pose.pose.orientation.x = msg->pose.orientation.x;
  pose.pose.orientation.y = msg->pose.orientation.y;
  pose.pose.orientation.z = msg->pose.orientation.z;
  
  poses.push_back(pose); // add the new pose to the poses vector

  uav_fly_msg.poses = poses; // assign the poses vector to the path message
  
  uav_fly_pub.publish(uav_fly_msg); // publish the path message
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "dji_visualization");
  ros::NodeHandle nh;

  // rviz 可视化消息发布
  v_loop_pub = nh.advertise<visualization_msgs::Marker>
                           ("/dji_uav/visualization/loop", 10);
  v_trajectory_pub = nh.advertise<nav_msgs::Path>
                           ("/dji_uav/visualization/trajectory", 10);
  uav_fly_pub = nh.advertise<nav_msgs::Path>
                           ("/dji_uav/visualization/fly", 10);
  
  // rviz 数据订阅，数据来源为perception感知层
  ros::Subscriber loop_pos_sub = nh.subscribe<dji_msgs::Loop>
                                 ("/dji_uav/loop_pos", 10,
                                  loop_pos_cb);

  ros::Subscriber trajectory_sub = nh.subscribe<dji_msgs::Trajectory>
                                   ("/dji_uav/trajectory", 1,
                                    trajectory_cb);
  
  ros::Subscriber fly_sub = nh.subscribe<geometry_msgs::PoseStamped>
                                    ("/dji_uav/fly", 1,
                                    fly_cb);
  ros::spin();
  return 0;
}
