#ifndef CAL_YAW_H
#define CAL_YAW_H

#include <Eigen/Eigen>
float d = 3;
Eigen::Vector3d get_yaw(float yaw)
{
  Eigen::Vector3d yaw_vec;
  yaw_vec[0] = cos(yaw * M_PI / 180.0);
  yaw_vec[1] = sin(yaw * M_PI / 180.0);
  yaw_vec[2] = 0.0;
  
  return yaw_vec;
}

// 根据圆环的偏航角yaw计算代表圆环朝向的单位向量
Eigen::Vector3d get_yaw_vec(float yaw)
{
  // 穿过圆环时的速度为3m/s，对应的速度方向为圆环的偏航角
  float velocity = 3.0;
  Eigen::Vector3d yaw_vec;
  yaw_vec[0] = cos(yaw * M_PI / 180.0);
  yaw_vec[1] = sin(yaw * M_PI / 180.0);
  yaw_vec[2] = 0.0;
  
  yaw_vec[0] = velocity * yaw_vec[0];
  yaw_vec[1] = velocity * yaw_vec[1];
  return yaw_vec;
}

#endif