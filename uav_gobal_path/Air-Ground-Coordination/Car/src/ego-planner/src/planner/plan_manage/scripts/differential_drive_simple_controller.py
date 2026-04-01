#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np

from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
from ego_planner.msg import Bspline
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion

# dynamic_reconfigure（需要你在本包下建一个 cfg 文件，下面有示例）
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.server import Server
from ego_planner.cfg import DiffDrivePIDConfig


class BSplineTrajectory(object):
    """简易 B-spline 封装：用 De Boor 算法按 knots 时间参数计算位置"""
    def __init__(self):
        self.order = None          # k (order = degree + 1)
        self.knots = None          # numpy array, shape (m,)
        self.ctrl_pts = None       # numpy array, shape (N,3)
        self.t_min = 0.0           # 有效时间段起点
        self.t_max = 0.0           # 有效时间段终点

    @staticmethod
    def de_boor(k, t, c, x):
        """
        k : order
        t : knot vector (len = n + k + 1)
        c : control points, shape (n+1, dim)
        x : parameter
        """
        t = np.asarray(t, dtype=float)
        c = np.asarray(c, dtype=float)
        n = len(c) - 1

        # 找 i，使得 t[i] <= x < t[i+1]，i ∈ [k-1, n]
        if x >= t[n+1]:
            i = n
        else:
            i = np.searchsorted(t, x) - 1

        i = max(k-1, min(i, n))
        # de Boor 初始化
        d = [c[j].copy() for j in range(i - k + 1, i + 1)]

        # r from 1 to k-1
        for r in range(1, k):
            for j in range(k-1, r-1, -1):
                denom = t[i + j + 1 - r] - t[i + j - k + 1]
                if denom == 0.0:
                    alpha = 0.0
                else:
                    alpha = (x - t[i + j - k + 1]) / denom
                d[j] = (1.0 - alpha) * d[j-1] + alpha * d[j]

        return d[k-1]

    def from_msg(self, msg):
        # msg.order, msg.knots, msg.pos_pts
        self.order = int(msg.order)
        self.knots = np.array(msg.knots, dtype=float)

        # 控制点
        n_pts = len(msg.pos_pts)
        self.ctrl_pts = np.zeros((n_pts, 3), dtype=float)
        for i, p in enumerate(msg.pos_pts):
            self.ctrl_pts[i, 0] = p.x
            self.ctrl_pts[i, 1] = p.y
            self.ctrl_pts[i, 2] = p.z

        # 有效参数区间（跟 C++ UniformBspline 类似）
        k = self.order
        self.t_min = self.knots[k-1]
        self.t_max = self.knots[len(self.ctrl_pts)]

    def duration(self):
        return max(0.0, self.t_max - self.t_min)

    def eval(self, t_query):
        """在真实时间偏移 t_cur 下的 B-spline 位置（世界系）"""
        if self.order is None or self.ctrl_pts is None:
            return np.zeros(3)

        # t_query 为从 t_min 开始的时间偏移
        param_t = self.t_min + np.clip(t_query, 0.0, self.duration())
        pos = BSplineTrajectory.de_boor(self.order, self.knots, self.ctrl_pts, param_t)
        return pos

    def sample_for_visualization(self, n_samples=100):
        if self.order is None or self.ctrl_pts is None:
            return []

        points = []
        for i in range(n_samples + 1):
            tau = float(i) / float(n_samples)
            param_t = self.t_min + tau * (self.t_max - self.t_min)
            p = BSplineTrajectory.de_boor(self.order, self.knots, self.ctrl_pts, param_t)
            pt = Point()
            pt.x, pt.y, pt.z = p[0], p[1], p[2]
            points.append(pt)
        return points


class DifferentialDriveBSplineController(object):
    def __init__(self):
        # === 初始参数（会被 rosparam & dynamic_reconfigure 覆盖） ===
        self.max_linear_vel  = rospy.get_param("~max_linear_vel", 1.0)
        self.max_angular_vel = rospy.get_param("~max_angular_vel", 2.0)
        self.position_tolerance = rospy.get_param("~position_tolerance", 0.1)
        self.yaw_tolerance      = rospy.get_param("~yaw_tolerance", 0.1)

        self.kp_pos = rospy.get_param("~position_pid/kp", 1.2)
        self.ki_pos = rospy.get_param("~position_pid/ki", 0.0)
        self.kd_pos = rospy.get_param("~position_pid/kd", 0.1)

        self.kp_yaw = rospy.get_param("~yaw_pid/kp", 1.6)
        self.ki_yaw = rospy.get_param("~yaw_pid/ki", 0.0)
        self.kd_yaw = rospy.get_param("~yaw_pid/kd", 0.2)

        self.dt = rospy.get_param("~control_dt", 0.1)

        # === 状态变量 ===
        self.current_pose = Pose()
        self.current_yaw = 0.0
        self.has_odom = False

        self.spline = BSplineTrajectory()
        self.receive_traj = False
        self.start_time = rospy.Time(0)

        self.go_flag = False

        # PID 内部状态
        self.pos_integral = 0.0
        self.pos_prev_error = 0.0
        self.yaw_integral = 0.0
        self.yaw_prev_error = 0.0
        self.first_run = True

        # 实际轨迹记录（可视化）
        self.actual_traj_points = []
        self.MAX_TRAJ_POINTS = 1000
        self.desired_traj_points = []

        # === 发布器 ===
        self.cmd_vel_pub = rospy.Publisher("/car/cmd_vel", Twist, queue_size=10)
        self.desired_traj_pub = rospy.Publisher("/desired_trajectory", Marker, queue_size=1)
        self.actual_traj_pub = rospy.Publisher("/actual_trajectory", Marker, queue_size=1)
        self.param_marker_pub = rospy.Publisher("/controller_params_marker", Marker, queue_size=1)

        # === 订阅器 ===
        rospy.Subscriber("/planning/bspline", Bspline, self.bspline_callback)
        rospy.Subscriber("/car/Odometry", Odometry, self.odom_callback)
        rospy.Subscriber("ego_planner_node/go_flag", Int16, self.go_flag_callback)

        # === 动态参数调节 ===
        self.dyn_server = Server(DiffDrivePIDConfig, self.dynamic_reconfig_callback)

        # === 定时器 ===
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.control_loop)

        rospy.logwarn("DifferentialDriveBSplineController is ready.")
        rospy.loginfo("  - Using true B-spline time (knots) for trajectory following.")
        rospy.loginfo("  - Use rqt_reconfigure to tune PID and speed limits.")
        rospy.loginfo("  - View /desired_trajectory, /actual_trajectory and /controller_params_marker in RViz.")

    # ========== 动态参数回调 ==========
    def dynamic_reconfig_callback(self, config, level):
        # 这里把 dynamic_reconfigure 中的参数同步到控制器
        self.max_linear_vel  = config.max_linear_vel
        self.max_angular_vel = config.max_angular_vel
        self.position_tolerance = config.position_tolerance
        self.yaw_tolerance      = config.yaw_tolerance

        self.kp_pos = config.kp_pos
        self.ki_pos = config.ki_pos
        self.kd_pos = config.kd_pos

        self.kp_yaw = config.kp_yaw
        self.ki_yaw = config.ki_yaw
        self.kd_yaw = config.kd_yaw

        rospy.loginfo_throttle(2.0,
            "DynReconf: v_max=%.2f, w_max=%.2f | PosPID(%.2f,%.2f,%.2f) YawPID(%.2f,%.2f,%.2f)",
            self.max_linear_vel, self.max_angular_vel,
            self.kp_pos, self.ki_pos, self.kd_pos,
            self.kp_yaw, self.ki_yaw, self.kd_yaw
        )

        return config

    # ========== 回调函数 ==========
    def go_flag_callback(self, msg):
        new_go_flag = (msg.data != 0)
        if new_go_flag != self.go_flag:
            self.go_flag = new_go_flag
            if not self.go_flag:
                self.reset_pid()
                rospy.loginfo("Motion stopped - PID reset.")
            else:
                rospy.loginfo("Motion started.")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        q = self.current_pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw
        self.has_odom = True

        # 记录实际轨迹
        pt = Point()
        pt.x = self.current_pose.position.x
        pt.y = self.current_pose.position.y
        pt.z = self.current_pose.position.z
        self.actual_traj_points.append(pt)
        if len(self.actual_traj_points) > self.MAX_TRAJ_POINTS:
            self.actual_traj_points.pop(0)

    def bspline_callback(self, msg):
        # 从消息构造 B-spline
        self.spline.from_msg(msg)

        # 使用 knots 的时间范围作为轨迹时长
        duration = self.spline.duration()
        self.start_time = msg.start_time
        if self.start_time.to_sec() == 0.0:
            self.start_time = rospy.Time.now()

        self.receive_traj = True
        self.reset_pid()

        # 预采样期望轨迹点用于 RViz 可视化
        self.desired_traj_points = self.spline.sample_for_visualization(150)
        self.actual_traj_points = []  # 清空实际轨迹

        # rospy.loginfo("Received Bspline trajectory: order=%d, ctrl_pts=%d, duration=%.2f s",
        #               self.spline.order, self.spline.ctrl_pts.shape[0], duration)

    # ========== 主控制循环 ==========
    def control_loop(self, event):
        cmd = Twist()

        # 条件不满足 -> 停车
        if (not self.receive_traj) or (not self.go_flag) or (not self.has_odom):
            self.cmd_vel_pub.publish(cmd)
            # 仍然发布可视化 Marker
            self.publish_markers()
            return

        t_cur = (rospy.Time.now() - self.start_time).to_sec()
        duration = self.spline.duration()

        # 目标位置
        target = self.spline.eval(t_cur)
        target_point = Point()
        target_point.x, target_point.y, target_point.z = target[0], target[1], target[2]

        # ===== 计算误差 =====
        dx = target[0] - self.current_pose.position.x
        dy = target[1] - self.current_pose.position.y
        pos_error = math.sqrt(dx*dx + dy*dy)

        # 目标 yaw
        target_yaw = math.atan2(dy, dx)
        yaw_error = target_yaw - self.current_yaw
        yaw_error = (yaw_error + math.pi) % (2.0 * math.pi) - math.pi

        # ===== 终点处理 =====
        if (t_cur >= duration) and (pos_error < self.position_tolerance):
            rospy.loginfo_throttle(2.0, "Trajectory completed - arrived at goal.")
            self.cmd_vel_pub.publish(cmd)
            self.publish_markers()
            return

        # ===== PID 计算 =====
        if self.first_run:
            self.pos_prev_error = pos_error
            self.yaw_prev_error = yaw_error
            self.first_run = False

        # 位置 PID
        self.pos_integral += pos_error * self.dt
        pos_derivative = (pos_error - self.pos_prev_error) / self.dt
        linear_cmd = (self.kp_pos * pos_error +
                      self.ki_pos * self.pos_integral +
                      self.kd_pos * pos_derivative)

        # 航向 PID
        self.yaw_integral += yaw_error * self.dt
        yaw_derivative = (yaw_error - self.yaw_prev_error) / self.dt
        angular_cmd = (self.kp_yaw * yaw_error +
                       self.ki_yaw * self.yaw_integral +
                       self.kd_yaw * yaw_derivative)

        self.pos_prev_error = pos_error
        self.yaw_prev_error = yaw_error

        # 角度误差大时，先转向
        if abs(yaw_error) > self.yaw_tolerance:
            angle_factor = max(0.1, 1.0 - 2.0 * abs(yaw_error) / math.pi)
            linear = linear_cmd * angle_factor
        else:
            linear = linear_cmd

        angular = angular_cmd

        # 限幅
        linear = max(-self.max_linear_vel, min(self.max_linear_vel, linear))
        angular = max(-self.max_angular_vel, min(self.max_angular_vel, angular))

        cmd.linear.x = linear
        cmd.angular.z = angular

        self.cmd_vel_pub.publish(cmd)

        rospy.loginfo_throttle(
            1.0,
            "CMD v=%.3f, w=%.3f | err_pos=%.3f, err_yaw=%.3f | t=%.2f/%.2f",
            linear, angular, pos_error, yaw_error, t_cur, duration
        )

        self.publish_markers(target_point)

    # ========== Marker 可视化 ==========
    def publish_markers(self, current_target=None):
        # 期望轨迹
        if self.desired_traj_points:
            marker = Marker()
            marker.header.frame_id = "camera_init"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "desired_trajectory"
            marker.id = 0
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.03  # 线宽
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.points = self.desired_traj_points
            self.desired_traj_pub.publish(marker)

        # 实际轨迹
        if self.actual_traj_points:
            marker = Marker()
            marker.header.frame_id = "camera_init"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "actual_trajectory"
            marker.id = 1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.03
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.points = self.actual_traj_points
            self.actual_traj_pub.publish(marker)

        # 参数文字 Marker，用来“可视化当前参数”
        marker = Marker()
        marker.header.frame_id = "camera_init"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "controller_params"
        marker.id = 2
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # 放在原点附近
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.0
        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.3  # 字体大小
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.9

        marker.text = (
            "v_max={:.2f}, w_max={:.2f}\n"
            "PosPID: Kp={:.2f}, Ki={:.2f}, Kd={:.2f}\n"
            "YawPID: Kp={:.2f}, Ki={:.2f}, Kd={:.2f}"
        ).format(
            self.max_linear_vel, self.max_angular_vel,
            self.kp_pos, self.ki_pos, self.kd_pos,
            self.kp_yaw, self.ki_yaw, self.kd_yaw
        )

        self.param_marker_pub.publish(marker)

    # ========== 工具 ==========
    def reset_pid(self):
        self.pos_integral = 0.0
        self.pos_prev_error = 0.0
        self.yaw_integral = 0.0
        self.yaw_prev_error = 0.0
        self.first_run = True


def main():
    rospy.init_node("differential_drive_bspline_controller")
    node = DifferentialDriveBSplineController()
    rospy.spin()

if __name__ == "__main__":
    main()
