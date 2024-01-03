#! /usr/bin/python3.8

import numpy as np
import casadi as ca
import time

# ROS
import rospy

from dji_msgs.msg import Trajectory
from geometry_msgs.msg import PoseStamped
from airsim_ros.msg import AngleRateThrottle

import os, sys

BASEPATH = os.path.abspath(__file__).split('script', 1)[0]+'script/function_model/'
sys.path += [BASEPATH]

from quadrotor_control import QuadrotorSimpleModel
from tracker import TrackerMPC

class Traj():
    def __init__(self, traj : Trajectory):
        poss = []
        yaws = []
        ts = []
        for i, pos in enumerate(traj.pos):
            poss.append([pos.x, pos.y, pos.z])
            yaws.append(traj.yaw[i])
            ts.append(traj.time[i])

        self._poss = np.array(poss)
        self._yaws = np.array(yaws)
        self._N = self._poss.shape[0]
        if self._N < 2:
            return
        dir = self._poss[1 : ] - self._poss[ : -1]
        self._dir_norm = np.linalg.norm(dir, axis = 1)
        self._u_dir = dir/self._dir_norm[:, np.newaxis]
        self._ts = np.array(ts)

    def sample(self, pos, dt, N):
        # calculate t0 and idx0
        t0 = 0
        idx0 = 0
        
        pos = np.array(pos)
        dl = np.linalg.norm(self._poss - pos, axis=1)
        min_idx = np.argmin(dl)
        if min_idx == 0:
            idx0 = min_idx
            d_v = pos - self._poss[0]
            u_dir = self._u_dir[0]
            u_t =  np.dot(d_v, u_dir) / self._dir_norm[0]
            if u_t < 0:
                t0 = self._ts[0]
            else:
                t0 = u_t * (self._ts[1] - self._ts[0]) + self._ts[0]
        else:
            idx0 = min_idx - 1
            d_v = pos - self._poss[idx0]
            u_dir = self._u_dir[idx0]
            u_t = np.dot(d_v, u_dir) / self._dir_norm[idx0]
            if u_t > 1:
                idx0 = idx0 + 1
                if idx0 == self._N - 1:
                    t0 = self._ts[-1]
                else:
                    d_v = pos - self._poss[idx0]
                    u_dir = self._u_dir[idx0]
                    u_t = np.dot(d_v, u_dir) / self._dir_norm[idx0]
                    if u_t < 0:
                        t0 = self._ts[idx0]
                    else:
                        t0 = u_t * (self._ts[idx0 + 1] - self._ts[idx0]) + self._ts[idx0]
            else:
                t0 = u_t * (self._ts[idx0 + 1] - self._ts[idx0]) + self._ts[idx0]

        # sample N points
        ts = np.linspace(t0 + dt, t0 + dt * N, N)
        idx = idx0
        poss = []
        yaws = []
        for t in ts:
            while idx + 1 < self._N and t > self._ts[idx + 1]:
                idx += 1
            if idx == self._N - 1:
                poss.append(self._poss[-1])
                yaws.append(self._yaws[-1])
                continue
            u_dir = self._u_dir[idx]
            u_t = (t - self._ts[idx]) / (self._ts[idx + 1] - self._ts[idx])
            poss.append(self._poss[idx] + u_t * self._dir_norm[idx] * u_dir)
            yaws.append(self._yaws[idx] + u_t * (self._yaws[idx + 1] - self._yaws[idx]))
        
        return np.array(poss), np.array(yaws), ts

def quat_mul(q1:np.array, q2:np.array):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    q = np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    ])
    return q
# 四元数旋转向量
# q: [qw, qx, qy, qz]
# v: [vx, vy, vz]
def quat_rot_vector(q:np.array, v:np.array):
    q = q / np.linalg.norm(q)
    q_inv = np.array([q[0], -q[1], -q[2], -q[3]])
    qv = quat_mul(quat_mul(q, np.concatenate([[0], v])), q_inv)
    return qv[1 : ]

def cal_v(x, dt):
    v = np.array([0, 0, 0])
    return v
trajectory = None
rospy.init_node("tracking")

setpoint_raw_pub = rospy.Publisher("~angle_rate_throttle_frame", AngleRateThrottle, queue_size=1, tcp_nodelay=True)
quad = QuadrotorSimpleModel(BASEPATH+'quad/quad_px4.yaml')
tracker = TrackerMPC(quad)

# tracker.define_opt()
tracker.load_so(BASEPATH+"generated/track_mpc.so")

def odom_cb(msg: PoseStamped):
    ## control
    u = AngleRateThrottle()
    # u.rollRate = 0
    # u.pitchRate = 0
    # u.yawRate = 0
    # u.throttle = 0
    
    # calculate v
    # v = cal_v(msg.pose.position, 0.1)
    # 
    # if trajectory != None:
    #     q = -np.array([msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])
    #     # v_b = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
    #     # v = quat_rot_vector(q, v_b) # v in inertial frame
    #     # w = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])
    #     p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        
    #     x0 = np.concatenate([p, v, q])
        
    #     poss, yaws, ts = trajectory.sample(p, 0.1, 5)
    #     print("##############################################")
    #     print(poss)
    #     print(p, v, q)
        
    #     res = tracker.solve(x0, poss.reshape(-1), yaws.reshape(-1))
    #     x = res['x'].full().flatten()
    #     Tt = x[tracker._Herizon*10+0]
    #     wx = x[tracker._Herizon*10+1]
    #     wy = x[tracker._Herizon*10+2]
    #     wz = x[tracker._Herizon*10+3]
        
    # publish
    u.rollRate = 0
    u.pitchRate = 0
    u.yawRate = 0
    u.throttle = 1
    setpoint_raw_pub.publish(u)
    print(u.throttle, u.rollRate, u.pitchRate, u.yawRate)
        
def track_traj_cb(msg: Trajectory):
    global trajectory
    trajectory = Traj(msg)

rospy.Subscriber("~odom", PoseStamped, odom_cb, queue_size=1, tcp_nodelay=True)
rospy.Subscriber("~track_traj", Trajectory, track_traj_cb, queue_size=1, tcp_nodelay=True)

rospy.spin()
