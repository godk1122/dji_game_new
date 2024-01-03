#include <ros/ros.h>
#include <dji_msgs/Loop.h>
#include <dji_msgs/Trajectory.h>
#include <airsim_ros/CirclePoses.h>
#include <airsim_ros/Circle.h>
#include <Eigen/Eigen>

// 圆环消息订阅与发布
ros::Publisher loop_pub;
ros::Subscriber loop_sub;

// 回调函数处理
dji_msgs::Loop loop_camera;

// 感知层数据处理，发布更新后的圆环全局坐标与偏航角
// // //待处理：
void loop_cb(const airsim_ros::CirclePoses::ConstPtr& msg) {
  for(int i = 0 ; i <= 16; i++)
  {
    loop_camera.loop_id = msg->poses[i].index;
    loop_camera.loop_pos.x = msg->poses[i].position.x;
    loop_camera.loop_pos.y = msg->poses[i].position.y;
    loop_camera.loop_pos.z = msg->poses[i].position.z;
    loop_camera.loop_yaw = msg->poses[i].yaw;
    loop_pub.publish(loop_camera);
  }
}
// // //


int main(int argc, char *argv[]) {
  ros::init(argc, argv, "dji_perception");
  ros::NodeHandle nh;
  
  // 订阅airsim圆环参考位置信息
  // 根据需要remap话题，修改launch文件，话题映射为yolo检测出的圆环位置姿态信息
  ros::Subscriber loop_sub = nh.subscribe("/airsim_node/drone_1/debug/circle_poses_gt", 10, loop_cb);

  // 消息发布
  loop_pub = nh.advertise<dji_msgs::Loop>("/dji_uav/loop_pos", 10);
  
  ros::spin();
  return 0;
} 