#ifndef PLANNER_C_H
#define PLANNER_C_H

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
/*Task C   -- Mission for 3 yellow pillar -- traj init  */
Eigen::Vector3d start_pos_C,start_vel_C,start_acc_C;
Eigen::Vector3d mid_pos1_C,mid_vel1_C,mid_acc1_C;
Eigen::Vector3d mid_pos2_C,mid_vel2_C,mid_acc2_C;
Eigen::Vector3d end_pos_C,end_vel_C,end_acc_C;

Eigen::Vector3d pos_BC,vel_BC,acc_BC;

auto trajC_0 = PolynomialTraj::one_segment_traj_gen(pos_BC, vel_BC, acc_BC,
                                                  start_pos_C, start_vel_C, start_acc_C, 3.0);
auto trajC_1 = PolynomialTraj::one_segment_traj_gen(start_pos_C, start_vel_C, start_acc_C,
                                                  mid_pos1_C, mid_vel1_C, mid_acc1_C, 3.0);
auto trajC_2 = PolynomialTraj::one_segment_traj_gen(mid_pos1_C, mid_vel1_C, mid_acc1_C,
                                                  mid_pos2_C, mid_vel2_C, mid_acc2_C, 3.0);
auto trajC_3 = PolynomialTraj::one_segment_traj_gen(mid_pos2_C, mid_vel2_C, mid_acc2_C,
                                                  end_pos_C, end_vel_C, end_acc_C, 3.0); 
//
Eigen::Vector3d yawC_vec0, yawC_vec1, yawC_vec2, end_vec_C;
float start_yaw_C, taskC_yaw1, taskC_yaw2, end_yaw_C; 

//
void planner_C(const dji_msgs::Loop::ConstPtr& msg) 
{
    //初始点
    // 任务C的起始点
    if (msg->loop_id == 12)
    {
        // A mid position1;
        start_pos_C[0] = msg->loop_pos.x;
        start_pos_C[1] = msg->loop_pos.y;
        start_pos_C[2] = msg->loop_pos.z;
        start_yaw_C = msg->loop_yaw;
        
        // 路径点0的速度向量
        yawC_vec0 = get_yaw_vec(start_yaw_C);
        start_vel_C = yawC_vec0;
        start_vel_C[0] = -start_vel_C[0];
        start_vel_C[1] = -start_vel_C[1];
        // 路径点1的加速度向量
        start_acc_C = Eigen::Vector3d(0.0, 0.0, 0.0);

        //
        pos_BC[0] = msg->loop_pos.x + 15;
        pos_BC[1] = msg->loop_pos.y;
        pos_BC[2] = msg->loop_pos.z - 5;
        //
        vel_BC = start_vel_C;
        // 路径点3的加速度向量
        acc_BC = Eigen::Vector3d(0.0, 0.0, 0.0);
    }
    else if (msg->loop_id == 13)
    {
        // A mid position1;
        mid_pos1_C[0] = msg->loop_pos.x;
        mid_pos1_C[1] = msg->loop_pos.y;
        mid_pos1_C[2] = msg->loop_pos.z;
        taskC_yaw1 = msg->loop_yaw;

        // 路径点1的速度向量
        yawC_vec1 = get_yaw_vec(taskC_yaw1);
        mid_vel1_C = yawC_vec1;
        // mid_vel1_C[0] = -mid_vel1_C[0];
        // mid_vel1_C[1] = -mid_vel1_C[1];
        // 路径点1的加速度向量
        mid_acc1_C = Eigen::Vector3d(0.0, 0.0, 0.0);
    }
    else if (msg->loop_id == 14)
    {
        // A mid position2;
        mid_pos2_C[0] = msg->loop_pos.x;
        mid_pos2_C[1] = msg->loop_pos.y;
        mid_pos2_C[2] = msg->loop_pos.z;
        taskC_yaw2 = msg->loop_yaw;
        // 路径点2的速度向量
        yawC_vec2 = get_yaw_vec(taskC_yaw2);
        mid_vel2_C = yawC_vec2;
        // mid_vel2_C[0] = -mid_vel2_C[0];
        // mid_vel2_C[1] = -mid_vel2_C[1];
        // 路径点2的加速度向量
        mid_acc2_C = Eigen::Vector3d(0.0, 0.0, 0.0);
    }
    else
    {
    // do nothing
    }
    // C end position;
    end_pos_C = Eigen::Vector3d(-25.0, 120.0, -38.0);
    end_vel_C = Eigen::Vector3d(  2.0,   0.0,   0.0);
    end_acc_C = Eigen::Vector3d(  0.0,   0.0,   0.0);
    
}

#endif