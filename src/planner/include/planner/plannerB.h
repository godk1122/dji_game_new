#ifndef PLANNER_B_H
#define PLANNER_B_H

#include <iostream>
#include <string.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <traj_utils/polynomial_traj.h>
#include <traj_utils/planning_visualization.h>
#include <dji_msgs/Trajectory.h>
#include <dji_msgs/Loop.h>
#include <vector>
#include <std_msgs/Int8.h>
#include <bspline_opt/bspline_optimizer.h>
#include <dji_msgs/Trajectory.h>

#include "planner/cal_yaw.h"
// Trajectory point define 

//
/*Task B   -- Mission for 3 yellow pillar -- traj init  */
Eigen::Vector3d start_pos_B,start_vel_B,start_acc_B;
Eigen::Vector3d mid_pos1_B,mid_vel1_B,mid_acc1_B;
Eigen::Vector3d mid_pos2_B,mid_vel2_B,mid_acc2_B;
Eigen::Vector3d end_pos_B,end_vel_B,end_acc_B;

auto trajB_1 = PolynomialTraj::one_segment_traj_gen(start_pos_B, start_vel_B, start_acc_B,
                                                  mid_pos1_B, mid_vel1_B, mid_acc1_B, 3.0);
auto trajB_2 = PolynomialTraj::one_segment_traj_gen(mid_pos1_B, mid_vel1_B, mid_acc1_B,
                                                  mid_pos2_B, mid_vel2_B, mid_acc2_B, 3.0);
auto trajB_3 = PolynomialTraj::one_segment_traj_gen(mid_pos2_B, mid_vel2_B, mid_acc2_B,
                                                  end_pos_B, end_vel_B, end_acc_B, 3.0);                                                
//
Eigen::Vector3d yawB_vec0, yawB_vec1, yawB_vec2, end_vec_B;
float start_yaw_B, taskB_yaw1, taskB_yaw2, end_yaw_B; 

//
void planner_B(const dji_msgs::Loop::ConstPtr& msg) 
{
    //初始点
    // 任务A的起始点
    if (msg->loop_id == 5)
    {
        // A mid position1;
        start_pos_B[0] = msg->loop_pos.x;
        start_pos_B[1] = msg->loop_pos.y;
        start_pos_B[2] = msg->loop_pos.z;
        start_yaw_B = msg->loop_yaw;
        
        // 路径点0的速度向量
        yawB_vec0 = get_yaw_vec(start_yaw_B);
        start_vel_B = yawB_vec0;
        start_vel_B[0] = -start_vel_B[0];
        start_vel_B[1] = -start_vel_B[1];
        // 路径点1的加速度向量
        start_acc_B = Eigen::Vector3d(0.0, 0.0, 0.0);
    }
    else if (msg->loop_id == 6)
    {
        // A mid position1;
        mid_pos1_B[0] = msg->loop_pos.x;
        mid_pos1_B[1] = msg->loop_pos.y;
        mid_pos1_B[2] = msg->loop_pos.z;
        taskB_yaw1 = msg->loop_yaw;

        // 路径点1的速度向量
        yawB_vec1 = get_yaw_vec(taskB_yaw1);
        mid_vel1_B = yawB_vec1;
        mid_vel1_B[0] = -mid_vel1_B[0];
        mid_vel1_B[1] = -mid_vel1_B[1];
        // 路径点1的加速度向量
        mid_acc1_B = Eigen::Vector3d(0.0, 0.0, 0.0);
    }
    else if (msg->loop_id == 8)
    {
        // A mid position2;
        mid_pos2_B[0] = msg->loop_pos.x;
        mid_pos2_B[1] = msg->loop_pos.y;
        mid_pos2_B[2] = msg->loop_pos.z;
        taskB_yaw2 = msg->loop_yaw;
        // 路径点2的速度向量
        yawB_vec2 = get_yaw_vec(taskB_yaw2);
        mid_vel2_B = yawB_vec2;
        mid_vel2_B[0] = -mid_vel2_B[0];
        mid_vel2_B[1] = -mid_vel2_B[1];
        // 路径点2的加速度向量
        mid_acc2_B = Eigen::Vector3d(0.0, 0.0, 0.0);
    }
    else if (msg->loop_id == 12)
    {
        // A end position;
        end_pos_B[0] = msg->loop_pos.x + 15;
        end_pos_B[1] = msg->loop_pos.y;
        end_pos_B[2] = msg->loop_pos.z - 5;
        end_yaw_B = msg->loop_yaw;
        // 路径点3的速度向量
        end_vec_B = get_yaw_vec(end_yaw_B);
        end_vel_B = end_vec_B;
        end_vel_B[0] = -end_vel_B[0];
        end_vel_B[1] = -end_vel_B[1];
        // 路径点3的加速度向量
        end_acc_B = Eigen::Vector3d(0.0, 0.0, 0.0);
    }
    else
    {
    // do nothing
    }   
}

#endif