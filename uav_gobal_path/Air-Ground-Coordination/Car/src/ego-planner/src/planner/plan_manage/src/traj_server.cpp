#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "ego_planner/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Int16.h"
#include "visualization_msgs/Marker.h"

#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <algorithm>
#include <cmath>
#include <memory>

using ego_planner::UniformBspline;

// --------------------- 全局 ROS 句柄与发布器 ---------------------

ros::Publisher pos_cmd_pub;
ros::Publisher cmd_vel_pub;
ros::Publisher control_point_state_pub;

// --------------------- 轨迹相关变量 ---------------------

bool receive_traj_ = false;
std::vector<UniformBspline> traj_;  // 0: pos, 1: vel, 2: acc
double traj_duration_ = 0.0;
ros::Time start_time_;
int traj_id_ = 0;

// --------------------- 机器人 & 控制参数 ---------------------

struct DifferentialDriveParams {
  double max_linear_vel   = 1.0;   // 最大线速度 (m/s)
  double max_angular_vel  = 2.0;   // 最大角速度 (rad/s)
  double max_linear_acc   = 1.0;   // 最大线加速度 (m/s²)
  double max_angular_acc  = 2.0;   // 最大角加速度 (rad/s²)

  double position_tolerance = 0.05; // 位置容差 (m)
  double yaw_tolerance      = 0.10; // 角度容差 (rad) -- 允许前进的偏航误差
  double yaw_align_thresh   = 0.70; // 原地转向阈值 (rad) -- 大于此只转向

  double wheelbase = 0.5;          // 轴距 (m) 预留，不直接使用

  struct {
    double kp = 1.0, ki = 0.0, kd = 0.1;  // 位置 PID
  } position_pid;

  struct {
    double kp = 3.0, ki = 0.0, kd = 0.2;  // 角度 PID
  } yaw_pid;
};

DifferentialDriveParams robot_params;

// --------------------- 当前状态 ---------------------

geometry_msgs::Pose  current_pose;
geometry_msgs::Twist current_velocity;
double current_yaw = 0.0;

bool go_flag = false;  // 来自上位控制（例如 /ego_planner_node/go_flag）

// --------------------- PID 控制器 ---------------------

class PIDController {
public:
  PIDController(double kp, double ki, double kd, double dt, double integral_max = 1.0)
      : kp_(kp), ki_(ki), kd_(kd),
        dt_(dt), integral_max_(integral_max),
        integral_(0.0), prev_error_(0.0), first_run_(true) {}

  double compute(double error) {
    if (first_run_) {
      prev_error_ = error;
      first_run_  = false;
    }

    // P
    double p = kp_ * error;

    // I
    integral_ += error * dt_;
    integral_ = std::max(-integral_max_, std::min(integral_max_, integral_));
    double i = ki_ * integral_;

    // D
    double d = kd_ * (error - prev_error_) / dt_;
    prev_error_ = error;

    return p + i + d;
  }

  void reset() {
    integral_   = 0.0;
    prev_error_ = 0.0;
    first_run_  = true;
  }

  void setGains(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

private:
  double kp_, ki_, kd_;
  double dt_;
  double integral_max_;

  double integral_;
  double prev_error_;
  bool   first_run_;
};

// --------------------- 差速控制器：核心差速运动学逻辑 ---------------------

class DifferentialDriveController {
public:
  DifferentialDriveController(const DifferentialDriveParams& params, double dt)
      : params_(params),
        dt_(dt),
        pos_pid_(params.position_pid.kp,
                 params.position_pid.ki,
                 params.position_pid.kd,
                 dt, 1.0),
        yaw_pid_(params.yaw_pid.kp,
                 params.yaw_pid.ki,
                 params.yaw_pid.kd,
                 dt, 1.0) {}

  void reset() {
    pos_pid_.reset();
    yaw_pid_.reset();
  }

  geometry_msgs::Twist computeCommand(
      const geometry_msgs::Pose& current_pose,
      double current_yaw,
      const Eigen::Vector3d& target_pos,
      const Eigen::Vector3d& target_vel) {

    geometry_msgs::Twist cmd;
    cmd.linear.x  = 0.0;
    cmd.linear.y  = 0.0;
    cmd.linear.z  = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;

    // ----------------- 位置误差 -----------------
    double dx = target_pos.x() - current_pose.position.x;
    double dy = target_pos.y() - current_pose.position.y;
    double dist_err = std::sqrt(dx*dx + dy*dy);

    // 终点判断
    if (dist_err < params_.position_tolerance) {
      return cmd; // 停车
    }

    // ----------------- 角度误差 -----------------
    // 目标朝向：优先使用轨迹切线，如果轨迹速度很小，则用指向目标点的方向
    double vx_d = target_vel.x();
    double vy_d = target_vel.y();
    double target_yaw;

    if (std::fabs(vx_d) + std::fabs(vy_d) > 1e-3) {
      target_yaw = std::atan2(vy_d, vx_d);
    } else {
      target_yaw = std::atan2(dy, dx);
    }

    double yaw_err = target_yaw - current_yaw;
    while (yaw_err >  M_PI) yaw_err -= 2.0 * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;

    // ----------------- 三段式角度控制 -----------------
    const double yaw_align_thresh = params_.yaw_align_thresh;  // 原地转向阈值
    const double yaw_move_thresh  = params_.yaw_tolerance;     // 允许带速度前进的阈值

    if (std::fabs(yaw_err) > yaw_align_thresh) {
      // ① 偏差很大：只转向，不前进（典型差速“原地掉头”）
      double w_cmd = yaw_pid_.compute(yaw_err);
      w_cmd = clamp(w_cmd, -params_.max_angular_vel, params_.max_angular_vel);

      cmd.linear.x  = 0.0;
      cmd.angular.z = w_cmd;

    } else if (std::fabs(yaw_err) > yaw_move_thresh) {
      // ② 中等误差：慢速前进 + 转向，线速度根据角度误差打折
      double v_cmd = pos_pid_.compute(dist_err);
      double w_cmd = yaw_pid_.compute(yaw_err);

      double k = (std::fabs(yaw_err) - yaw_move_thresh) /
                 (yaw_align_thresh - yaw_move_thresh); // 0~1
      k = clamp(k, 0.0, 1.0);
      double angle_factor = 1.0 - k; // yaw_err 越大，angle_factor 越小

      cmd.linear.x  = v_cmd * angle_factor;
      cmd.angular.z = w_cmd;

    } else {
      // ③ 角度误差很小：正常跟踪轨迹 + 前馈
      double v_cmd = pos_pid_.compute(dist_err);
      double w_cmd = yaw_pid_.compute(yaw_err);

      cmd.linear.x  = v_cmd;
      cmd.angular.z = w_cmd;

      // 前馈线速度：当角度对得差不多时，适度跟随规划速度
      double v_ff = std::sqrt(vx_d*vx_d + vy_d*vy_d);
      if (v_ff > 1e-3) {
        // 不减小 PID 的结果，只保证不会比前馈小太多
        cmd.linear.x = std::max(cmd.linear.x, 0.5 * v_ff);
      }
    }

    // 速度限幅
    cmd.linear.x  = clamp(cmd.linear.x,  -params_.max_linear_vel,  params_.max_linear_vel);
    cmd.angular.z = clamp(cmd.angular.z, -params_.max_angular_vel, params_.max_angular_vel);

    return cmd;
  }

private:
  template<typename T>
  static T clamp(T v, T lo, T hi) {
    return std::max(lo, std::min(hi, v));
  }

  DifferentialDriveParams params_;
  double dt_;

  PIDController pos_pid_;
  PIDController yaw_pid_;
};

// --------------------- 速度平滑器（加速度限幅） ---------------------

class VelocitySmoother {
public:
  VelocitySmoother(double max_lin_acc, double max_ang_acc, double dt)
      : max_lin_acc_(max_lin_acc),
        max_ang_acc_(max_ang_acc),
        dt_(dt) {
    last_cmd_.linear.x  = 0.0;
    last_cmd_.angular.z = 0.0;
  }

  geometry_msgs::Twist smooth(const geometry_msgs::Twist& raw_cmd,
                              const DifferentialDriveParams& params) {
    geometry_msgs::Twist out = raw_cmd;

    // 线加速度限幅
    double lin_acc = (raw_cmd.linear.x - last_cmd_.linear.x) / dt_;
    if (lin_acc > max_lin_acc_) {
      out.linear.x = last_cmd_.linear.x + max_lin_acc_ * dt_;
    } else if (lin_acc < -max_lin_acc_) {
      out.linear.x = last_cmd_.linear.x - max_lin_acc_ * dt_;
    }

    // 角加速度限幅
    double ang_acc = (raw_cmd.angular.z - last_cmd_.angular.z) / dt_;
    if (ang_acc > max_ang_acc_) {
      out.angular.z = last_cmd_.angular.z + max_ang_acc_ * dt_;
    } else if (ang_acc < -max_ang_acc_) {
      out.angular.z = last_cmd_.angular.z - max_ang_acc_ * dt_;
    }

    // 再做一次速度限幅，确保安全
    out.linear.x  = clamp(out.linear.x,  -params.max_linear_vel,  params.max_linear_vel);
    out.angular.z = clamp(out.angular.z, -params.max_angular_vel, params.max_angular_vel);

    last_cmd_ = out;
    return out;
  }

  void reset() {
    last_cmd_.linear.x  = 0.0;
    last_cmd_.angular.z = 0.0;
  }

private:
  template<typename T>
  static T clamp(T v, T lo, T hi) {
    return std::max(lo, std::min(hi, v));
  }

  geometry_msgs::Twist last_cmd_;
  double max_lin_acc_;
  double max_ang_acc_;
  double dt_;
};

// --------------------- 全局控制器实例 ---------------------

constexpr double kControlDt = 0.01; // 100Hz

std::unique_ptr<DifferentialDriveController> g_controller;
std::unique_ptr<VelocitySmoother>           g_smoother;

// --------------------- 回调函数 ---------------------

void goFlagCallback(const std_msgs::Int16::ConstPtr& msg) {
  bool new_flag = (msg->data != 0);
  if (new_flag != go_flag) {
    go_flag = new_flag;
    if (!go_flag) {
      if (g_controller) g_controller->reset();
      if (g_smoother)   g_smoother->reset();
      ROS_INFO("Motion stopped, controller reset.");
    } else {
      ROS_INFO("Motion started.");
    }
  }
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  current_pose.position    = msg->pose.pose.position;
  current_pose.orientation = msg->pose.pose.orientation;
  current_velocity         = msg->twist.twist;

  current_yaw = tf::getYaw(msg->pose.pose.orientation);

  // 发布控制点状态（可用于 RViz）
  nav_msgs::Odometry cp_state = *msg;
  control_point_state_pub.publish(cp_state);
}

void bsplineCallback(const ego_planner::Bspline::ConstPtr& msg) {
  if (msg->pos_pts.empty() || msg->knots.empty()) {
    ROS_WARN("Received empty B-spline message, ignoring.");
    return;
  }

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
  Eigen::VectorXd knots(msg->knots.size());

  for (size_t i = 0; i < msg->knots.size(); ++i) {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i) {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());

  traj_duration_ = traj_[0].getTimeSum();
  start_time_    = msg->start_time;
  traj_id_       = msg->traj_id;
  receive_traj_  = true;

  if (g_controller) g_controller->reset();

  ROS_INFO("Received new B-spline trajectory, duration = %.3f s, id = %d",
           traj_duration_, traj_id_);
}

// --------------------- 主控制定时器回调（100Hz） ---------------------

void cmdCallback(const ros::TimerEvent&) {
  geometry_msgs::Twist cmd;
  cmd.linear.x  = 0.0;
  cmd.linear.y  = 0.0;
  cmd.linear.z  = 0.0;
  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;
  cmd.angular.z = 0.0;

  quadrotor_msgs::PositionCommand pos_cmd;
  ros::Time now = ros::Time::now();

  // 未收到轨迹或未允许运动：平滑停止
  if (!receive_traj_ || !go_flag || !g_controller || !g_smoother) {
    cmd = g_smoother ? g_smoother->smooth(cmd, robot_params) : cmd;
    cmd_vel_pub.publish(cmd);
    return;
  }

  double t_cur = 1.3;

  Eigen::Vector3d target_pos(0, 0, 0), target_vel(0, 0, 0), target_acc(0, 0, 0);

  if (t_cur >= 0.0 && t_cur <= traj_duration_) {
    // 在轨迹时间范围内：跟踪
    target_pos = traj_[0].evaluateDeBoorT(t_cur);
    target_vel = traj_[1].evaluateDeBoorT(t_cur);
    target_acc = traj_[2].evaluateDeBoorT(t_cur);

    cmd = g_controller->computeCommand(current_pose, current_yaw,
                                       target_pos, target_vel);
  } else if (t_cur > traj_duration_) {
    // 轨迹结束：向终点收敛
    target_pos = traj_[0].evaluateDeBoorT(traj_duration_);
    target_vel.setZero();
    target_acc.setZero();

    double dx = target_pos.x() - current_pose.position.x;
    double dy = target_pos.y() - current_pose.position.y;
    double dist_end = std::sqrt(dx*dx + dy*dy);

    if (dist_end > robot_params.position_tolerance) {
      cmd = g_controller->computeCommand(current_pose, current_yaw,
                                         target_pos, target_vel);
    } else {
      cmd.linear.x  = 0.0;
      cmd.angular.z = 0.0;
      ROS_INFO_THROTTLE(2.0, "Trajectory finished and target reached.");
    }
  } else {
    // 轨迹未到开始时间
    cmd.linear.x  = 0.0;
    cmd.angular.z = 0.0;
    ROS_INFO_THROTTLE(2.0, "Waiting for trajectory start, t=%.3f", t_cur);
  }

  // 速度平滑
  cmd = g_smoother->smooth(cmd, robot_params);

  // 发布 cmd_vel
  cmd_vel_pub.publish(cmd);

  // 发布 PositionCommand 方便可视化 / 其他模块
  pos_cmd.header.stamp    = now;
  pos_cmd.header.frame_id = "map";
  pos_cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  pos_cmd.trajectory_id   = traj_id_;

  pos_cmd.position.x = target_pos.x();
  pos_cmd.position.y = target_pos.y();
  pos_cmd.position.z = target_pos.z();

  pos_cmd.velocity.x = target_vel.x();
  pos_cmd.velocity.y = target_vel.y();
  pos_cmd.velocity.z = target_vel.z();

  pos_cmd.acceleration.x = target_acc.x();
  pos_cmd.acceleration.y = target_acc.y();
  pos_cmd.acceleration.z = target_acc.z();

  pos_cmd_pub.publish(pos_cmd);

  // 调试用误差打印
  double dist_err = 0.0;
  double yaw_err  = 0.0;
  if (t_cur >= 0.0 && t_cur <= traj_duration_) {
    double dx = target_pos.x() - current_pose.position.x;
    double dy = target_pos.y() - current_pose.position.y;
    dist_err  = std::sqrt(dx*dx + dy*dy);

    double target_yaw = std::atan2(dy, dx);
    yaw_err = target_yaw - current_yaw;
    while (yaw_err >  M_PI) yaw_err -= 2.0 * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;
  }

  ROS_INFO_THROTTLE(1.0,
                    "cmd: v=%.3f m/s, w=%.3f rad/s | err: pos=%.3f m, yaw=%.3f rad",
                    cmd.linear.x, cmd.angular.z, dist_err, yaw_err);
}

// --------------------- main ---------------------

int main(int argc, char** argv) {
  ros::init(argc, argv, "differential_drive_pid_traj_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // 读取参数
  pnh.param("max_linear_vel",  robot_params.max_linear_vel,  1.0);
  pnh.param("max_angular_vel", robot_params.max_angular_vel, 2.0);
  pnh.param("max_linear_acc",  robot_params.max_linear_acc,  1.0);
  pnh.param("max_angular_acc", robot_params.max_angular_acc, 2.0);

  pnh.param("position_tolerance", robot_params.position_tolerance, 0.05);
  pnh.param("yaw_tolerance",      robot_params.yaw_tolerance,      0.30);
  pnh.param("yaw_align_thresh",   robot_params.yaw_align_thresh,   0.70);
  pnh.param("wheelbase",          robot_params.wheelbase,          0.5);

  pnh.param("position_pid/kp", robot_params.position_pid.kp, 2.0);
  pnh.param("position_pid/ki", robot_params.position_pid.ki, 0.0);
  pnh.param("position_pid/kd", robot_params.position_pid.kd, 0.1);

  pnh.param("yaw_pid/kp", robot_params.yaw_pid.kp, 6.0);
  pnh.param("yaw_pid/ki", robot_params.yaw_pid.ki, 0.0);
  pnh.param("yaw_pid/kd", robot_params.yaw_pid.kd, 0.2);

  ROS_INFO("=== Differential Drive Trajectory Server Parameters ===");
  ROS_INFO("Max v = %.2f m/s, Max w = %.2f rad/s",
           robot_params.max_linear_vel, robot_params.max_angular_vel);
  ROS_INFO("Max a_v = %.2f m/s^2, Max a_w = %.2f rad/s^2",
           robot_params.max_linear_acc, robot_params.max_angular_acc);
  ROS_INFO("Pos tol = %.3f m, Yaw tol = %.3f rad, Align thresh = %.3f rad",
           robot_params.position_tolerance,
           robot_params.yaw_tolerance,
           robot_params.yaw_align_thresh);
  ROS_INFO("Position PID: Kp=%.2f, Ki=%.2f, Kd=%.2f",
           robot_params.position_pid.kp,
           robot_params.position_pid.ki,
           robot_params.position_pid.kd);
  ROS_INFO("Yaw PID:      Kp=%.2f, Ki=%.2f, Kd=%.2f",
           robot_params.yaw_pid.kp,
           robot_params.yaw_pid.ki,
           robot_params.yaw_pid.kd);

  // 创建控制器 & 平滑器
  g_controller = std::make_unique<DifferentialDriveController>(robot_params, kControlDt);
  g_smoother   = std::make_unique<VelocitySmoother>(robot_params.max_linear_acc,
                                                    robot_params.max_angular_acc,
                                                    kControlDt);

  // 订阅
  ros::Subscriber bspline_sub = nh.subscribe("planning/bspline", 10, bsplineCallback);
  ros::Subscriber odom_sub    = nh.subscribe("car/Odometry",     10, odometryCallback);
  ros::Subscriber go_flag_sub = nh.subscribe("ego_planner_node/go_flag", 50, goFlagCallback);

  // 发布
  pos_cmd_pub           = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
  cmd_vel_pub           = nh.advertise<geometry_msgs::Twist>("/car/cmd_vel", 10);
  control_point_state_pub = nh.advertise<nav_msgs::Odometry>("/control_point_state", 10);

  // 控制定时器
  ros::Timer cmd_timer = nh.createTimer(ros::Duration(kControlDt), cmdCallback);

  ROS_WARN("Differential Drive B-spline Trajectory Server Ready.");
  ROS_INFO("Subscribing:");
  ROS_INFO("  - planning/bspline");
  ROS_INFO("  - car/Odometry");
  ROS_INFO("  - ego_planner_node/go_flag");
  ROS_INFO("Publishing:");
  ROS_INFO("  - car/cmd_vel");
  ROS_INFO("  - position_cmd");
  ROS_INFO("  - control_point_state");

  ros::spin();
  return 0;
}
