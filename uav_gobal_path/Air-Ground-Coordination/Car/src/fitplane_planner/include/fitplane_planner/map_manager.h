#ifndef FITPLANE_PLANNER_MAP_MANAGER_H
#define FITPLANE_PLANNER_MAP_MANAGER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <string>

namespace fitplane_planner {

// 地图栅格结构体
struct MapCell {
    int8_t occupancy;      // 0 = free, 100 = occupied, -1 = unknown
    ros::Time last_updated; // 上次更新时间戳，用于防乱序

    MapCell() : occupancy(-1), last_updated(0) {}
};

class MapManager {
public:
    MapManager(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    ~MapManager();

private:
    // 回调函数
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void rawMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void updateLoop(const ros::TimerEvent& event);

    // 内部处理函数
    void clearFarCells();     // 清除距离机器人太远的栅格
    void publishFusedMap();   // 发布最终地图

    // ROS 句柄
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // 订阅与发布
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_raw_map_;
    ros::Publisher  pub_fused_map_;
    ros::Timer      update_timer_;

    // 地图参数
    std::string world_frame_;
    double map_resolution_;
    double map_width_meters_;
    double map_height_meters_;
    double map_update_frequency_;
    
    // 维护参数
    double max_keep_distance_;
    bool enable_distance_filter_;

    // 地图数据
    int map_width_cells_;
    int map_height_cells_;
    std::vector<MapCell> main_map_; // 我们的主地图数据
    geometry_msgs::Point map_origin_; // 地图左下角的世界坐标

    // 机器人状态
    geometry_msgs::Pose current_robot_pose_;
    bool robot_pose_received_ = false;
};

} // namespace fitplane_planner

#endif // FITPLANE_PLANNER_MAP_MANAGER_H