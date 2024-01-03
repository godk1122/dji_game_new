#ifndef PLANNER_D_H
#define PLANNER_D_H

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
/*Task D   -- Mission for 2 special pillar -- traj init  */
Eigen::Vector3d pos_CD, vel_CD, acc_CD;
Eigen::Vector3d start_pos_D,start_vel_D,start_acc_D;
Eigen::Vector3d pos_D_mid, vel_D_mid, acc_D_mid;
Eigen::Vector3d end_pos_D,end_vel_D,end_acc_D;
Eigen::Vector3d end_pos_all,end_vel_all,end_acc_all;


auto trajD_0 = PolynomialTraj::one_segment_traj_gen(pos_CD, vel_CD, acc_CD,
                                                  start_pos_D, start_vel_D, start_acc_D, 3.0);
auto trajD_1 = PolynomialTraj::one_segment_traj_gen(start_pos_D, start_vel_D, start_acc_D,
                                                  pos_D_mid, vel_D_mid, acc_D_mid, 3.0);
auto trajD_2 = PolynomialTraj::one_segment_traj_gen(pos_D_mid, vel_D_mid, acc_D_mid,
                                                  end_pos_D,end_vel_D,end_acc_D, 3.0);
auto trajD_3 = PolynomialTraj::one_segment_traj_gen(end_pos_D,end_vel_D,end_acc_D,  
                                                  end_pos_all,end_vel_all,end_acc_all, 3.0);

//
Eigen::Vector3d start_vec_D, end_vec_D;
float start_yaw_D, end_yaw_D; 

//
void planner_D(const dji_msgs::Loop::ConstPtr& msg) 
{
    //初始点
    // 任务D的起始点
    pos_CD = Eigen::Vector3d(-25.0, 120.0, -38.0);
    vel_CD = Eigen::Vector3d(2.0, 0.0, 0.0);
    acc_CD = Eigen::Vector3d(0.0, 0.0, 0.0);

    //
    if (msg->loop_id == 15)
    {
        // A mid position1;
        start_pos_D[0] = msg->loop_pos.x;
        start_pos_D[1] = msg->loop_pos.y;
        start_pos_D[2] = msg->loop_pos.z;
        start_yaw_D = msg->loop_yaw;
        
        // 路径点1的速度向量
        start_vec_D = get_yaw_vec(start_yaw_D);
        start_vel_D = start_vec_D;

        // 路径点1的加速度向量
        start_acc_D = Eigen::Vector3d(0.0, 0.0, 0.0);

        //
        pos_D_mid = Eigen::Vector3d(40.0, 88.0, -6.0);
        vel_D_mid = Eigen::Vector3d(1.0, 0.0, 0.0);
        acc_D_mid = Eigen::Vector3d(0.0, 0.0, 0.0);
    }
    
    else if (msg->loop_id == 16)
    {
        // A mid position1;
        end_pos_D[0] = msg->loop_pos.x;
        end_pos_D[1] = msg->loop_pos.y;
        end_pos_D[2] = msg->loop_pos.z;
        end_yaw_D = msg->loop_yaw;

        // 路径点2的速度向量
        end_vec_D = get_yaw_vec(end_yaw_D);
        end_vel_D = end_vec_D;
        // mid_vel1_C[0] = -mid_vel1_C[0];
        // mid_vel1_C[1] = -mid_vel1_C[1];
        // 路径点2的加速度向量
        end_acc_D = Eigen::Vector3d(0.0, 0.0, 0.0);
    }

    else
    {
    // do nothing
    }   

    end_pos_all = Eigen::Vector3d(60.0, 88.0, -6.0);
    end_vel_all = Eigen::Vector3d(0.0, 0.0, 0.0);
    end_acc_all = Eigen::Vector3d(0.0, 0.0, 0.0);
}

#endif