#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "ros/package.h"
#include "ros/ros.h"
#include "std_msgs/ColorRGBA.h"
#include "visualization_msgs/Marker.h"

using namespace std;

static constexpr double kPi = 3.14159265358979323846;

static string mesh_resource;
static bool resolve_mesh_resource = true;
static double color_r, color_g, color_b, color_a;
static double cov_scale, scale, rotate_yaw;
static string output_frame;
static int robot_id;

static bool cov_pos = false;
static bool cov_vel = false;
static bool origin = false;
static bool isOriginSet = false;
static Eigen::Isometry3d poseOrigin = Eigen::Isometry3d::Identity();

static double path_append_dt = 0.1;
static int path_max_points = 0;  // 0 means unlimited
static double traj_append_dt = 0.5;
static int traj_max_points = 0;  // 0 means unlimited

static ros::Publisher posePub;
static ros::Publisher pathPub;
static ros::Publisher velPub;
static ros::Publisher covPub;
static ros::Publisher covVelPub;
static ros::Publisher trajPub;
static ros::Publisher meshPub;

static geometry_msgs::PoseStamped poseROS;
static nav_msgs::Path pathROS;
static visualization_msgs::Marker velROS;
static visualization_msgs::Marker covROS;
static visualization_msgs::Marker covVelROS;
static visualization_msgs::Marker trajROS;
static visualization_msgs::Marker meshROS;

static inline int markerId()
{
  return robot_id >= 0 ? robot_id : 0;
}

static inline string resolvedFrame(const std_msgs::Header& header)
{
  if (!output_frame.empty())
    return output_frame;
  if (!header.frame_id.empty() && header.frame_id != "null")
    return header.frame_id;
  ROS_WARN_THROTTLE(1.0, "drone_visualization: odom.header.frame_id is empty; defaulting marker frame to 'odom' (set ~output_frame to override).");
  return "odom";
}

static Eigen::Quaterniond quaternionFromMsg(const geometry_msgs::Quaternion& q)
{
  Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
  if (quat.norm() == 0.0)
    quat.w() = 1.0;
  quat.normalize();
  return quat;
}

static void rpyFromQuaternion(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw)
{
  const Eigen::Matrix3d R = q.toRotationMatrix();
  yaw = std::atan2(R(1, 0), R(0, 0));
  pitch = std::atan2(-R(2, 0), std::sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2)));
  roll = std::atan2(R(2, 1), R(2, 2));
}

static Eigen::Quaterniond quaternionFromRpy(double roll, double pitch, double yaw)
{
  const Eigen::AngleAxisd yawAA(yaw, Eigen::Vector3d::UnitZ());
  const Eigen::AngleAxisd pitchAA(pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd rollAA(roll, Eigen::Vector3d::UnitX());
  Eigen::Quaterniond q = yawAA * pitchAA * rollAA;
  q.normalize();
  return q;
}

static Eigen::Isometry3d isometryFromPoseMsg(const geometry_msgs::Pose& pose)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  T.linear() = quaternionFromMsg(pose.orientation).toRotationMatrix();
  return T;
}

static void capVectorSize(std::vector<geometry_msgs::PoseStamped>& vec, int max_size)
{
  if (max_size <= 0)
    return;
  if (static_cast<int>(vec.size()) <= max_size)
    return;
  vec.erase(vec.begin(), vec.begin() + (vec.size() - max_size));
}

template <typename T>
static void capVectorSize(std::vector<T>& vec, int max_size)
{
  if (max_size <= 0)
    return;
  if (static_cast<int>(vec.size()) <= max_size)
    return;
  vec.erase(vec.begin(), vec.begin() + (vec.size() - max_size));
}

static std::string resolvePackageResourceToFile(const std::string& uri)
{
  constexpr const char* kPrefix = "package://";
  if (uri.rfind(kPrefix, 0) != 0)
    return uri;

  const std::string rest = uri.substr(std::string(kPrefix).size());
  const std::string::size_type slash = rest.find('/');
  if (slash == std::string::npos)
    return uri;

  const std::string pkg = rest.substr(0, slash);
  const std::string rel = rest.substr(slash + 1);
  const std::string pkg_path = ros::package::getPath(pkg);
  if (pkg_path.empty())
    return uri;

  return std::string("file://") + pkg_path + "/" + rel;
}

static void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  if ((msg->header.frame_id.empty() || msg->header.frame_id == string("null")) && output_frame.empty())
  {
    ROS_WARN_THROTTLE(1.0,
                      "drone_visualization: ignoring odom because header.frame_id is empty/'null' and ~output_frame is empty; set ~output_frame (e.g. 'world').");
    return;
  }

  Eigen::Isometry3d T = isometryFromPoseMsg(msg->pose.pose);
  Eigen::Quaterniond q_pose(T.linear());
  q_pose.normalize();

  Eigen::Vector3d vel(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);

  if (origin && !isOriginSet)
  {
    isOriginSet = true;
    poseOrigin = T;
  }
  if (origin)
  {
    const Eigen::Matrix3d R_world = T.linear();
    const Eigen::Vector3d vel_body = R_world.transpose() * vel;

    T = poseOrigin.inverse() * T;
    const Eigen::Matrix3d R_world_rel = T.linear();
    vel = R_world_rel * vel_body;
    q_pose = Eigen::Quaterniond(R_world_rel);
    q_pose.normalize();
  }

  const string frame = resolvedFrame(msg->header);
  ROS_DEBUG_THROTTLE(2.0, "drone_visualization: publishing markers in frame '%s' from odom frame '%s'", frame.c_str(), msg->header.frame_id.c_str());

  // Pose
  poseROS.header = msg->header;
  poseROS.header.stamp = msg->header.stamp;
  poseROS.header.frame_id = frame;
  poseROS.pose.position.x = T.translation().x();
  poseROS.pose.position.y = T.translation().y();
  poseROS.pose.position.z = T.translation().z();
  poseROS.pose.orientation.w = q_pose.w();
  poseROS.pose.orientation.x = q_pose.x();
  poseROS.pose.orientation.y = q_pose.y();
  poseROS.pose.orientation.z = q_pose.z();
  posePub.publish(poseROS);

  // Velocity
  const double yaw_vel = std::atan2(vel.y(), vel.x());
  const double pitch_vel = -std::atan2(vel.z(), std::hypot(vel.x(), vel.y()));
  const Eigen::Quaterniond q_vel = quaternionFromRpy(0.0, pitch_vel, yaw_vel);
  velROS.header.frame_id = frame;
  velROS.header.stamp = msg->header.stamp;
  velROS.ns = string("velocity");
  velROS.id = markerId();
  velROS.type = visualization_msgs::Marker::ARROW;
  velROS.action = visualization_msgs::Marker::ADD;
  velROS.pose.position.x = T.translation().x();
  velROS.pose.position.y = T.translation().y();
  velROS.pose.position.z = T.translation().z();
  velROS.pose.orientation.w = q_vel.w();
  velROS.pose.orientation.x = q_vel.x();
  velROS.pose.orientation.y = q_vel.y();
  velROS.pose.orientation.z = q_vel.z();
  velROS.scale.x = vel.norm();
  velROS.scale.y = 0.05;
  velROS.scale.z = 0.05;
  velROS.color.a = 1.0;
  velROS.color.r = color_r;
  velROS.color.g = color_g;
  velROS.color.b = color_b;
  velPub.publish(velROS);

  // Path
  static ros::Time prevt = msg->header.stamp;
  if ((msg->header.stamp - prevt).toSec() > path_append_dt)
  {
    prevt = msg->header.stamp;
    pathROS.header = poseROS.header;
    pathROS.poses.push_back(poseROS);
    capVectorSize(pathROS.poses, path_max_points);
    pathPub.publish(pathROS);
  }

  // Covariance Position
  if (cov_pos)
  {
    Eigen::Matrix3d P = Eigen::Matrix3d::Zero();
    for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++)
        P(r, c) = msg->pose.covariance[r * 6 + c];
    P = 0.5 * (P + P.transpose());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(P);
    Eigen::Vector3d eigVal = solver.eigenvalues();
    Eigen::Matrix3d eigVec = solver.eigenvectors();
    if (eigVec.determinant() < 0)
    {
      for (int k = 0; k < 3; k++)
      {
        Eigen::Matrix3d eigVecRev = eigVec;
        eigVecRev.col(k) *= -1;
        if (eigVecRev.determinant() > 0)
        {
          eigVec = eigVecRev;
          break;
        }
      }
    }

    covROS.header.frame_id = frame;
    covROS.header.stamp = msg->header.stamp;
    covROS.ns = string("covariance");
    covROS.id = markerId();
    covROS.type = visualization_msgs::Marker::SPHERE;
    covROS.action = visualization_msgs::Marker::ADD;
    covROS.pose.position.x = T.translation().x();
    covROS.pose.position.y = T.translation().y();
    covROS.pose.position.z = T.translation().z();
    const Eigen::Quaterniond q_cov(eigVec);
    covROS.pose.orientation.w = q_cov.w();
    covROS.pose.orientation.x = q_cov.x();
    covROS.pose.orientation.y = q_cov.y();
    covROS.pose.orientation.z = q_cov.z();
    covROS.scale.x = std::sqrt(std::max(0.0, eigVal(0))) * cov_scale;
    covROS.scale.y = std::sqrt(std::max(0.0, eigVal(1))) * cov_scale;
    covROS.scale.z = std::sqrt(std::max(0.0, eigVal(2))) * cov_scale;
    covROS.color.a = 0.4;
    covROS.color.r = color_r * 0.5;
    covROS.color.g = color_g * 0.5;
    covROS.color.b = color_b * 0.5;
    covPub.publish(covROS);
  }

  // Covariance Velocity
  if (cov_vel)
  {
    Eigen::Matrix3d P = Eigen::Matrix3d::Zero();
    for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++)
        P(r, c) = msg->twist.covariance[r * 6 + c];
    P = 0.5 * (P + P.transpose());

    const Eigen::Matrix3d R = q_pose.toRotationMatrix();
    P = R * P * R.transpose();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(P);
    Eigen::Vector3d eigVal = solver.eigenvalues();
    Eigen::Matrix3d eigVec = solver.eigenvectors();
    if (eigVec.determinant() < 0)
    {
      for (int k = 0; k < 3; k++)
      {
        Eigen::Matrix3d eigVecRev = eigVec;
        eigVecRev.col(k) *= -1;
        if (eigVecRev.determinant() > 0)
        {
          eigVec = eigVecRev;
          break;
        }
      }
    }

    covVelROS.header.frame_id = frame;
    covVelROS.header.stamp = msg->header.stamp;
    covVelROS.ns = string("covariance_velocity");
    covVelROS.id = markerId();
    covVelROS.type = visualization_msgs::Marker::SPHERE;
    covVelROS.action = visualization_msgs::Marker::ADD;
    covVelROS.pose.position.x = T.translation().x();
    covVelROS.pose.position.y = T.translation().y();
    covVelROS.pose.position.z = T.translation().z();
    const Eigen::Quaterniond q_cov(eigVec);
    covVelROS.pose.orientation.w = q_cov.w();
    covVelROS.pose.orientation.x = q_cov.x();
    covVelROS.pose.orientation.y = q_cov.y();
    covVelROS.pose.orientation.z = q_cov.z();
    covVelROS.scale.x = std::sqrt(std::max(0.0, eigVal(0))) * cov_scale;
    covVelROS.scale.y = std::sqrt(std::max(0.0, eigVal(1))) * cov_scale;
    covVelROS.scale.z = std::sqrt(std::max(0.0, eigVal(2))) * cov_scale;
    covVelROS.color.a = 0.4;
    covVelROS.color.r = color_r;
    covVelROS.color.g = color_g;
    covVelROS.color.b = color_b;
    covVelPub.publish(covVelROS);
  }

  // Trajectory (colored line strip, shares the same color as robot)
  static ros::Time pt = msg->header.stamp;
  const ros::Time t = msg->header.stamp;
  if ((t - pt).toSec() > traj_append_dt)
  {
    trajROS.header.frame_id = frame;
    trajROS.header.stamp = ros::Time::now();
    trajROS.ns = string("trajectory");
    trajROS.id = markerId();
    trajROS.type = visualization_msgs::Marker::LINE_STRIP;
    trajROS.action = visualization_msgs::Marker::ADD;
    trajROS.pose.orientation.w = 1;
    trajROS.scale.x = 0.1;
    trajROS.color.r = 0.0;
    trajROS.color.g = 1.0;
    trajROS.color.b = 0.0;
    trajROS.color.a = 0.8;

    geometry_msgs::Point p;
    p.x = T.translation().x();
    p.y = T.translation().y();
    p.z = T.translation().z();
    trajROS.points.push_back(p);

    std_msgs::ColorRGBA color;
    color.r = color_r;
    color.g = color_g;
    color.b = color_b;
    color.a = 1;
    trajROS.colors.push_back(color);

    capVectorSize(trajROS.points, traj_max_points);
    if (trajROS.colors.size() == trajROS.points.size())
      capVectorSize(trajROS.colors, traj_max_points);

    pt = t;
    trajPub.publish(trajROS);
  }

  // Mesh model
  meshROS.header.frame_id = frame;
  meshROS.header.stamp = msg->header.stamp;
  meshROS.ns = "robot";
  meshROS.id = markerId();
  meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
  meshROS.action = visualization_msgs::Marker::ADD;
  meshROS.pose.position.x = T.translation().x();
  meshROS.pose.position.y = T.translation().y();
  meshROS.pose.position.z = T.translation().z();

  double roll, pitch, yaw;
  rpyFromQuaternion(q_pose, roll, pitch, yaw);
  yaw += rotate_yaw * kPi / 180.0;
  const Eigen::Quaterniond q_mesh = quaternionFromRpy(roll, pitch, yaw);
  meshROS.pose.orientation.w = q_mesh.w();
  meshROS.pose.orientation.x = q_mesh.x();
  meshROS.pose.orientation.y = q_mesh.y();
  meshROS.pose.orientation.z = q_mesh.z();
  meshROS.scale.x = scale;
  meshROS.scale.y = scale;
  meshROS.scale.z = scale;
  meshROS.color.a = color_a;
  meshROS.color.r = color_r;
  meshROS.color.g = color_g;
  meshROS.color.b = color_b;
  if (resolve_mesh_resource)
  {
    const std::string resolved = resolvePackageResourceToFile(mesh_resource);
    if (resolved == mesh_resource && mesh_resource.rfind("package://", 0) == 0)
    {
      ROS_WARN_THROTTLE(2.0,
                        "drone_visualization: could not resolve mesh_resource '%s' (package not found); RViz may fail to load it",
                        mesh_resource.c_str());
    }
    meshROS.mesh_resource = resolved;
  }
  else
  {
    meshROS.mesh_resource = mesh_resource;
  }
  meshPub.publish(meshROS);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drone_visualization");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param("mesh_resource", mesh_resource, std::string("package://drone_visualization/meshes/yunque-M.dae"));
  pnh.param("resolve_mesh_resource", resolve_mesh_resource, true);
  pnh.param("color/r", color_r, 1.0);
  pnh.param("color/g", color_g, 0.0);
  pnh.param("color/b", color_b, 0.0);
  pnh.param("color/a", color_a, 1.0);
  pnh.param("origin", origin, false);
  pnh.param("robot_scale", scale, 1.0);
  pnh.param("rotate_yaw_deg", rotate_yaw, 0.0);

  pnh.param("covariance_scale", cov_scale, 100.0);
  pnh.param("covariance_position", cov_pos, false);
  pnh.param("covariance_velocity", cov_vel, false);

  pnh.param("path_append_dt", path_append_dt, 0.1);
  pnh.param("path_max_points", path_max_points, 0);
  pnh.param("traj_append_dt", traj_append_dt, 0.5);
  pnh.param("traj_max_points", traj_max_points, 0);

  // Preferred param name.
  if (!pnh.getParam("robot_id", robot_id))
  {
    // Compatibility with old configs.
    pnh.param("drone_id", robot_id, -1);
  }

  // Preferred param name.
  pnh.param("output_frame", output_frame, std::string(""));
  // Compatibility with old configs.
  if (output_frame.empty())
    pnh.getParam("frame_id", output_frame);

  const std::string odom_topic = nh.resolveName("odom");
  ROS_INFO("drone_visualization: subscribing to odom: %s", odom_topic.c_str());
  ROS_INFO("drone_visualization: output_frame: '%s'", output_frame.empty() ? "(use odom header.frame_id)" : output_frame.c_str());
  ros::Subscriber sub_odom = nh.subscribe("odom", 100, odom_callback);

  posePub = pnh.advertise<geometry_msgs::PoseStamped>("pose", 100, true);
  pathPub = pnh.advertise<nav_msgs::Path>("path", 100, true);
  velPub = pnh.advertise<visualization_msgs::Marker>("velocity", 100, true);
  covPub = pnh.advertise<visualization_msgs::Marker>("covariance", 100, true);
  covVelPub = pnh.advertise<visualization_msgs::Marker>("covariance_velocity", 100, true);
  trajPub = pnh.advertise<visualization_msgs::Marker>("trajectory", 100, true);
  meshPub = pnh.advertise<visualization_msgs::Marker>("robot", 100, true);

  ros::spin();
  return 0;
}
