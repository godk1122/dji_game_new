// #include <plan_env/grid_map.h>
// #include "planner/cal_yaw.h"
#include <dji_msgs/Loop.h>
#include "planner/plannerA.h"
#include "planner/plannerB.h"
#include "planner/plannerC.h"
#include "planner/plannerD.h"
// #include "planner/pos_param.h"

// GridMap::Ptr grid_map_;

// planning visualization
ego_planner::PlanningVisualization::Ptr visualization_;

// topic define
ros::Publisher traj_pub_;
// ros::Subscriber local_pos_sub;
ros::Subscriber task_sub;
// void param_init();
// trajectory
PolynomialTraj traj_;
dji_msgs::Trajectory traj_msg_;

// Posparam pos_param_;
static geometry_msgs::PoseStamped aim;

// message from perception
void loop_cb(const dji_msgs::Loop::ConstPtr& msg) {
    planner_A(msg); 
    planner_B(msg);
    planner_C(msg);
    planner_D(msg);
}

// update trajectory
static float time_all;
static float timeA_1, timeA_2, timeA_3, timeA_4, timeA_5, timeA_6, timeA_all;
static float timeB_1, timeB_2, timeB_3, timeBC, timeB_all;
static float timeC_1, timeC_2, timeCD, timeC_all;
static float timeD_1, timeD_2, timeD_3, timeD_end, timeD_all;
static float time_fb = 1;
void trajectory_update()
{   
    // ----------------------------------------------------
    timeA_1 = 8.0;
    timeA_2 = 8.0;
    timeA_3 = 10.0;
    timeA_4 = 7.0;
    timeA_5 = 12.0;
    timeA_6 = 6.0;
    timeA_all = timeA_1 + timeA_2 + timeA_3 + timeA_4 + timeA_5 + timeA_6 + 10 * time_fb;
    
    // ----------------------------------------------------
    // task B trajectory time ： 26
    timeB_1 = 4.0;
    timeB_2 = 12.0;
    timeB_3 = 10.0;
    timeB_all = timeB_1 + timeB_2 + timeB_3;

    // ----------------------------------------------------
    // task C trajectory time ： 29
    timeBC = 5.0;
    timeC_1 = 9.0;
    timeC_2 = 9.0;
    timeCD = 10.0;
    timeC_all = timeBC + timeC_1 + timeC_2 + timeCD;

    // ----------------------------------------------------
    // task D trajectory time ： 23
    timeD_1 = 4.0;
    timeD_2 = 12.0;
    timeD_3 = 4.0;
    timeD_end = 2.0;
    timeD_all = timeD_1 + timeD_2 + timeD_3 + timeD_end;
    // ----------------------------------------------------
    // all trajectory time : 128
    time_all = timeA_all + timeB_all  + timeC_all + timeD_all;
    // ----------------------------------------------------
    // 计算任务A的轨迹 利用五次多项式表示
    // trajA_0 = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc,
    //                                           start_pos_A, start_vel_A, start_acc_A, 50.0);
    // trajA_1 = PolynomialTraj::one_segment_traj_gen(start_pos_A, start_vel_A, start_acc_A,
    //                                                mid_pos1_A, mid_vel1_A, mid_acc1_A, timeA_1);
    // trajA_2 = PolynomialTraj::one_segment_traj_gen(mid_pos1_A, mid_vel1_A, mid_acc1_A,
    //                                                mid_pos2_A, mid_vel2_A, mid_acc2_A, timeA_2);
    // trajA_3 = PolynomialTraj::one_segment_traj_gen(mid_pos2_A, mid_vel2_A, mid_acc2_A,
    //                                                mid_pos3_A, mid_vel3_A, mid_acc3_A, timeA_3); 
    // trajA_4 = PolynomialTraj::one_segment_traj_gen(mid_pos3_A, mid_vel3_A, mid_acc3_A,
    //                                                mid_pos4_A, mid_vel4_A, mid_acc4_A, timeA_4); 
    // trajA_5 = PolynomialTraj::one_segment_traj_gen(mid_pos4_A, mid_vel4_A, mid_acc4_A,
    //                                                mid_pos5_A, mid_vel5_A, mid_acc5_A, timeA_5);                                                  
    // trajA_6 = PolynomialTraj::one_segment_traj_gen(mid_pos5_A, mid_vel5_A, mid_acc5_A,
    //                                                end_pos_A, end_vel_A, end_acc_A, timeA_6);
    //
    
    // A new
    trajA_1_m = PolynomialTraj::one_segment_traj_gen(start_pos_A, start_vel_A, start_acc_A,
                                                  pos1_A_f, vel1_A_f, acc1_A_f, timeA_1);
    trajA_1_b = PolynomialTraj::one_segment_traj_gen(pos1_A_f, vel1_A_f, acc1_A_f,
                                                  mid_pos1_A, mid_vel1_A, mid_acc1_A, time_fb);
    // trajA2
    trajA_2_f = PolynomialTraj::one_segment_traj_gen(mid_pos1_A, mid_vel1_A, mid_acc1_A,
                                                  pos1_A_b, vel1_A_b, acc1_A_b, time_fb);     
    trajA_2_m = PolynomialTraj::one_segment_traj_gen(pos1_A_b, vel1_A_b, acc1_A_b,
                                                  pos2_A_f, vel2_A_f, acc2_A_f, timeA_2);
    trajA_2_b = PolynomialTraj::one_segment_traj_gen(pos2_A_f, vel2_A_f, acc2_A_f,
                                                  mid_pos2_A, mid_vel2_A, mid_acc2_A, time_fb); 
    // trajA3
    trajA_3_f = PolynomialTraj::one_segment_traj_gen(mid_pos2_A, mid_vel2_A, mid_acc2_A,
                                                      pos2_A_b, vel2_A_b, acc2_A_b, time_fb);
    trajA_3_m = PolynomialTraj::one_segment_traj_gen(pos2_A_b, vel2_A_b, acc2_A_b,
                                                      pos3_A_f, vel3_A_f, acc3_A_f, timeA_3);
    trajA_3_b = PolynomialTraj::one_segment_traj_gen(pos3_A_f, vel3_A_f, acc3_A_f,
                                                          mid_pos3_A, mid_vel3_A, mid_acc3_A, time_fb);       
    // trajA4
    trajA_4_f = PolynomialTraj::one_segment_traj_gen(mid_pos3_A, mid_vel3_A, mid_acc3_A,
                                                      pos3_A_b, vel3_A_b, acc3_A_b, time_fb);
    trajA_4_m = PolynomialTraj::one_segment_traj_gen(pos3_A_b, vel3_A_b, acc3_A_b,
                                                      pos4_A_f, vel4_A_f, acc4_A_f, timeA_4);       
    trajA_4_b = PolynomialTraj::one_segment_traj_gen(pos4_A_f, vel4_A_f, acc4_A_f,
                                                      mid_pos4_A, mid_vel4_A, mid_acc4_A, time_fb);     
    // trajA5
    trajA_5_f = PolynomialTraj::one_segment_traj_gen(mid_pos4_A, mid_vel4_A, mid_acc4_A,
                                                      pos4_A_b, vel4_A_b, acc4_A_b, time_fb);
    trajA_5_m = PolynomialTraj::one_segment_traj_gen(pos4_A_b, vel4_A_b, acc4_A_b,
                                                      pos5_A_f, vel5_A_f, acc5_A_f, timeA_5);     
    trajA_5_b = PolynomialTraj::one_segment_traj_gen(pos5_A_f, vel5_A_f, acc5_A_f,
                                                      mid_pos5_A, mid_vel5_A, mid_acc5_A, time_fb);
    // trajA6
    trajA_6_f = PolynomialTraj::one_segment_traj_gen(mid_pos5_A, mid_vel5_A, mid_acc5_A,
                                                      pos5_A_b, vel5_A_b, acc5_A_b, time_fb);
    trajA_6_m = PolynomialTraj::one_segment_traj_gen(pos5_A_b, vel5_A_b, acc5_A_b,
                                                      end_pos_A, end_vel_A, end_acc_A, timeA_6);
    // 计算任务B的轨迹 利用五次多项式表示
    //
    trajB_1 = PolynomialTraj::one_segment_traj_gen(start_pos_B, start_vel_B, start_acc_B,
                                                   mid_pos1_B, mid_vel1_B, mid_acc1_B, timeB_1);
    trajB_2 = PolynomialTraj::one_segment_traj_gen(mid_pos1_B, mid_vel1_B, mid_acc1_B,
                                                   mid_pos2_B, mid_vel2_B, mid_acc2_B, timeB_2);
    trajB_3 = PolynomialTraj::one_segment_traj_gen(mid_pos2_B, mid_vel2_B, mid_acc2_B,  
                                                   end_pos_B, end_vel_B, end_acc_B, timeB_3);                                                                       
    //  
    // 计算任务C的轨迹 利用五次多项式表示
    //
    trajC_0 = PolynomialTraj::one_segment_traj_gen(pos_BC, vel_BC, acc_BC,
                                                   start_pos_C, start_vel_C, start_acc_C, timeBC);
    trajC_1 = PolynomialTraj::one_segment_traj_gen(start_pos_C, start_vel_C, start_acc_C,
                                                   mid_pos1_C, mid_vel1_C, mid_acc1_C, timeC_1);
    trajC_2 = PolynomialTraj::one_segment_traj_gen(mid_pos1_C, mid_vel1_C, mid_acc1_C,
                                                    mid_pos2_C, mid_vel2_C, mid_acc2_C, timeC_2);
    trajC_3 = PolynomialTraj::one_segment_traj_gen(mid_pos2_C, mid_vel2_C, mid_acc2_C,
                                                    end_pos_C, end_vel_C, end_acc_C, timeCD);  

    //  
    // 计算任务D的轨迹 利用五次多项式表示
    //
    trajD_0 = PolynomialTraj::one_segment_traj_gen(pos_CD, vel_CD, acc_CD,
                                                   start_pos_D, start_vel_D, start_acc_D, timeD_1);
    trajD_1 = PolynomialTraj::one_segment_traj_gen(start_pos_D, start_vel_D, start_acc_D,
                                                    pos_D_mid, vel_D_mid, acc_D_mid, timeD_2);
    trajD_2 = PolynomialTraj::one_segment_traj_gen(pos_D_mid, vel_D_mid, acc_D_mid,
                                                    end_pos_D,end_vel_D,end_acc_D, timeD_3);
    trajD_3 = PolynomialTraj::one_segment_traj_gen(end_pos_D,end_vel_D,end_acc_D,
                                                    end_pos_all,end_vel_all,end_acc_all, timeD_end);

}
void traj_init()
{
    /*  Task A traj init */                                      
    traj_.addSegment(trajA_1_m.getCoef(0)[0], trajA_1_m.getCoef(1)[0], trajA_1_m.getCoef(2)[0], trajA_1_m.getTimes()[0]);
    traj_.addSegment(trajA_1_b.getCoef(0)[0], trajA_1_b.getCoef(1)[0], trajA_1_b.getCoef(2)[0], trajA_1_b.getTimes()[0]);
   
    traj_.addSegment(trajA_2_f.getCoef(0)[0], trajA_2_f.getCoef(1)[0], trajA_2_f.getCoef(2)[0], trajA_2_f.getTimes()[0]);
    traj_.addSegment(trajA_2_m.getCoef(0)[0], trajA_2_m.getCoef(1)[0], trajA_2_m.getCoef(2)[0], trajA_2_m.getTimes()[0]);
    traj_.addSegment(trajA_2_b.getCoef(0)[0], trajA_2_b.getCoef(1)[0], trajA_2_b.getCoef(2)[0], trajA_2_b.getTimes()[0]);
   
    traj_.addSegment(trajA_3_f.getCoef(0)[0], trajA_3_f.getCoef(1)[0], trajA_3_f.getCoef(2)[0], trajA_3_f.getTimes()[0]);
    traj_.addSegment(trajA_3_m.getCoef(0)[0], trajA_3_m.getCoef(1)[0], trajA_3_m.getCoef(2)[0], trajA_3_m.getTimes()[0]);
    traj_.addSegment(trajA_3_b.getCoef(0)[0], trajA_3_b.getCoef(1)[0], trajA_3_b.getCoef(2)[0], trajA_3_b.getTimes()[0]);
   
    traj_.addSegment(trajA_4_f.getCoef(0)[0], trajA_4_f.getCoef(1)[0], trajA_4_f.getCoef(2)[0], trajA_4_f.getTimes()[0]);
    traj_.addSegment(trajA_4_m.getCoef(0)[0], trajA_4_m.getCoef(1)[0], trajA_4_m.getCoef(2)[0], trajA_4_m.getTimes()[0]);
    traj_.addSegment(trajA_4_b.getCoef(0)[0], trajA_4_b.getCoef(1)[0], trajA_4_b.getCoef(2)[0], trajA_4_b.getTimes()[0]);
   
    traj_.addSegment(trajA_5_f.getCoef(0)[0], trajA_5_f.getCoef(1)[0], trajA_5_f.getCoef(2)[0], trajA_5_f.getTimes()[0]);
    traj_.addSegment(trajA_5_m.getCoef(0)[0], trajA_5_m.getCoef(1)[0], trajA_5_m.getCoef(2)[0], trajA_5_m.getTimes()[0]);
    traj_.addSegment(trajA_5_b.getCoef(0)[0], trajA_5_b.getCoef(1)[0], trajA_5_b.getCoef(2)[0], trajA_5_b.getTimes()[0]);
   
    traj_.addSegment(trajA_6_f.getCoef(0)[0], trajA_6_f.getCoef(1)[0], trajA_6_f.getCoef(2)[0], trajA_6_f.getTimes()[0]);
    traj_.addSegment(trajA_6_m.getCoef(0)[0], trajA_6_m.getCoef(1)[0], trajA_6_m.getCoef(2)[0], trajA_6_m.getTimes()[0]);

    // traj_.addSegment(trajA_3.getCoef(0)[0], trajA_3.getCoef(1)[0], trajA_3.getCoef(2)[0], trajA_3.getTimes()[0]);
    // traj_.addSegment(trajA_4.getCoef(0)[0], trajA_4.getCoef(1)[0], trajA_4.getCoef(2)[0], trajA_4.getTimes()[0]);
    // traj_.addSegment(trajA_5.getCoef(0)[0], trajA_5.getCoef(1)[0], trajA_5.getCoef(2)[0], trajA_5.getTimes()[0]);
    // traj_.addSegment(trajA_6.getCoef(0)[0], trajA_6.getCoef(1)[0], trajA_6.getCoef(2)[0], trajA_6.getTimes()[0]);
    /*  Task B traj init */
    traj_.addSegment(trajB_1.getCoef(0)[0], trajB_1.getCoef(1)[0], trajB_1.getCoef(2)[0], trajB_1.getTimes()[0]);
    traj_.addSegment(trajB_2.getCoef(0)[0], trajB_2.getCoef(1)[0], trajB_2.getCoef(2)[0], trajB_2.getTimes()[0]);
    traj_.addSegment(trajB_3.getCoef(0)[0], trajB_3.getCoef(1)[0], trajB_3.getCoef(2)[0], trajB_3.getTimes()[0]);
    /*  Task C traj init */
    traj_.addSegment(trajC_0.getCoef(0)[0], trajC_0.getCoef(1)[0], trajC_0.getCoef(2)[0], trajC_0.getTimes()[0]);
    traj_.addSegment(trajC_1.getCoef(0)[0], trajC_1.getCoef(1)[0], trajC_1.getCoef(2)[0], trajC_1.getTimes()[0]);
    traj_.addSegment(trajC_2.getCoef(0)[0], trajC_2.getCoef(1)[0], trajC_2.getCoef(2)[0], trajC_2.getTimes()[0]);
    traj_.addSegment(trajC_3.getCoef(0)[0], trajC_3.getCoef(1)[0], trajC_3.getCoef(2)[0], trajC_3.getTimes()[0]);
    /*  Task D traj init */
    traj_.addSegment(trajD_0.getCoef(0)[0], trajD_0.getCoef(1)[0], trajD_0.getCoef(2)[0], trajD_0.getTimes()[0]);
    traj_.addSegment(trajD_1.getCoef(0)[0], trajD_1.getCoef(1)[0], trajD_1.getCoef(2)[0], trajD_1.getTimes()[0]);
    traj_.addSegment(trajD_2.getCoef(0)[0], trajD_2.getCoef(1)[0], trajD_2.getCoef(2)[0], trajD_2.getTimes()[0]);
    traj_.addSegment(trajD_3.getCoef(0)[0], trajD_3.getCoef(1)[0], trajD_3.getCoef(2)[0], trajD_3.getTimes()[0]);    
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "dji_planner_new_node");
  ros::NodeHandle nh("~");
  
  // 订阅任务A的轨迹点（发布者为perception感知层，根据airsim中提供的参考信息与视觉校正得到的数据）
  // 并根据新的轨迹点更新轨迹
  task_sub = nh.subscribe<dji_msgs::Loop>("/dji_uav/loop_pos", 50, loop_cb); 
  trajectory_update();

  // local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, local_pos_cb);
  traj_pub_ = nh.advertise<dji_msgs::Trajectory>("trajectory", 1);

  visualization_.reset(new ego_planner::PlanningVisualization(nh));  

  std::cout << "--------------------------" << std::endl;
  std::cout << "task A time: " << timeA_all << std::endl;
  std::cout << "task B time: " << timeB_all << std::endl;
  std::cout << "task C time: " << timeC_all << std::endl;
  std::cout << "task D time: " << timeD_all << std::endl;
  std::cout << "all time: " << time_all << std::endl;
  std::cout << "--------------------------" << std::endl;

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    std::vector<Eigen::Vector3d> pos_sampled;
    // sample traj pos
    pos_sampled.clear();
    traj_msg_.pos.clear();
    traj_msg_.yaw.clear();
    traj_msg_.time.clear();

    // int flag_mission = check_mission(local_pos);
    trajectory_update();
    
    // clear traj_
    traj_.reset();
    traj_init();
    //
    for (double t = 0.0; t < time_all; t += 0.1)
    {
      const auto& pt = traj_.evaluate(t);
      Eigen::Vector3d vel = traj_.evaluateVel(t);
      float yaw;
      pos_sampled.push_back(pt);

      traj_msg_.header.stamp = ros::Time::now();
      traj_msg_.header.frame_id = "world";
      geometry_msgs::Point point;
      point.x = pt.x();
      point.y = pt.y();
      point.z = pt.z();     
      traj_msg_.pos.push_back(point);
      //对轨迹的偏航角进行处理，使无人机的偏航方向与轨迹速度方向相同，确保圆环在无人机的视角中
      // yaw = get_vec_yaw(vel);
      // traj_msg_.yaw.push_back(yaw);
      if (t <= timeA_1 + timeA_2 + timeA_3 + 5 * time_fb)
      {
        traj_msg_.yaw.push_back(-0);
      }

      else if (t> timeA_1 + timeA_2 + timeA_3 + 5 * time_fb&& t <= timeA_1 + timeA_2 + timeA_3 + timeA_4 + 7 * time_fb)
      {
        traj_msg_.yaw.push_back(M_PI/4);
      }
      
      else if (t > timeA_1 + timeA_2 + timeA_3 + timeA_4 + 7 * time_fb && t <= timeA_all + timeB_all + timeBC)
      {
        traj_msg_.yaw.push_back(M_PI);
      }

      else if (t > timeA_all + timeB_all + timeBC && t <= timeA_all + timeB_all + timeC_all)
      {
        traj_msg_.yaw.push_back(M_PI/2.1);
      }

      else if (t > timeA_all + timeB_all + timeC_all && t <= timeA_all + timeB_all + timeC_all + timeD_1)
      {
        traj_msg_.yaw.push_back(-M_PI/4);
      }
        
      else if (t > timeA_all + timeB_all + timeC_all + timeD_1 && t <= time_all)
      {
        traj_msg_.yaw.push_back(0);
      }

      else
      {
        traj_msg_.yaw.push_back(0);
      }


      //
      traj_msg_.time.push_back(t);
      
    }
    visualization_->displayGlobalPathList(pos_sampled, 0.1, 1);
    traj_pub_.publish(traj_msg_);
    // std::cout << traj_msg_ << std::endl;

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

