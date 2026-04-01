#include "fitplane_planner/map_manager.h"
#include <cmath>

namespace {
    // 拼图逻辑：直接覆盖
    inline void stitchMeasurement(fitplane_planner::MapCell& cell, int8_t meas_occ) {
        // 1. 如果局部地图这一格是“未知”(-1)，绝对不要覆盖全局地图！
        //    因为局部地图边缘都是未知的，覆盖了会把已经建好的全局地图“擦除”成未知。
        if (meas_occ == -1) {
            return;
        }

        // 2. 只有确定的 Free(0) 或 Occupied(100) 才更新
        if (meas_occ >= 50) {
            cell.occupancy = 100;
        } else {
            cell.occupancy = 0;
        }
    }
} // namespace

namespace fitplane_planner {

MapManager::MapManager(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
    
    // 参数配置
    nh_private_.param("world_frame", world_frame_, std::string("map"));
    nh_private_.param("map_resolution", map_resolution_, 0.5);
    
    // 【关键修改】：建议把默认值设大一点，比如 500米 x 500米
    // 这样机器人无论怎么跑（只要不出这个范围），地图都会一直保留
    nh_private_.param("map_width_meters",  map_width_meters_,  500.0);
    nh_private_.param("map_height_meters", map_height_meters_, 500.0);
    
    nh_private_.param("map_update_frequency", map_update_frequency_, 2.0);

    // 计算栅格尺寸
    map_width_cells_  = static_cast<int>(map_width_meters_  / map_resolution_);
    map_height_cells_ = static_cast<int>(map_height_meters_ / map_resolution_);

    if (map_width_cells_ <= 0 || map_height_cells_ <= 0) {
        ROS_ERROR("MapManager: Map size invalid!");
        return;
    }

    // 分配内存（一旦分配，大小不再改变，原点不再移动）
    main_map_.resize(map_width_cells_ * map_height_cells_);

    sub_odom_    = nh_.subscribe("/odom", 1, &MapManager::odomCallback, this);
    sub_raw_map_ = nh_.subscribe("plane_OccMap", 1, &MapManager::rawMapCallback, this);
    pub_fused_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("/fused_map", 1, true); // latched=true

    update_timer_ = nh_.createTimer(
        ros::Duration(1.0 / map_update_frequency_),
        &MapManager::updateLoop, this
    );

    ROS_INFO("Global Map Stitcher Initialized: %.0fm x %.0fm (Resolution: %.2f)", 
             map_width_meters_, map_height_meters_, map_resolution_);
}

MapManager::~MapManager() {}

void MapManager::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_robot_pose_ = msg->pose.pose;

    if (!robot_pose_received_) {
        // 【关键】：地图初始化逻辑
        // 在第一次收到位置时，把当前位置设为地图的【正中心】
        // 这样机器人往前后左右各能跑 map_width/2 米而不出界
        map_origin_.x = current_robot_pose_.position.x - (map_width_cells_ * map_resolution_ * 0.5);
        map_origin_.y = current_robot_pose_.position.y - (map_height_cells_ * map_resolution_ * 0.5);
        map_origin_.z = 0.0;
        
        robot_pose_received_ = true;
        ROS_INFO("Map Anchor Set! Origin: (%.2f, %.2f). Robot is at center.", 
                 map_origin_.x, map_origin_.y);
    }
}

void MapManager::rawMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    if (!robot_pose_received_) return;

    const double msg_res = msg->info.resolution;
    const double origin_x = msg->info.origin.position.x;
    const double origin_y = msg->info.origin.position.y;
    const uint32_t width = msg->info.width;
    const uint32_t height = msg->info.height;
    const auto& data = msg->data;

    // 遍历局部地图的每一个点
    for (uint32_t y = 0; y < height; ++y) {
        double world_y = origin_y + (y + 0.5) * msg_res;
        
        // 算出这个点在【全局大地图】中的索引
        int my = static_cast<int>(std::floor((world_y - map_origin_.y) / map_resolution_));
        
        // 如果超出了全局地图的边界，就忽略（无法建图了）
        if (my < 0 || my >= map_height_cells_) continue;

        size_t main_row_offset = static_cast<size_t>(my) * map_width_cells_;
        size_t msg_row_offset = static_cast<size_t>(y) * width;

        for (uint32_t x = 0; x < width; ++x) {
            double world_x = origin_x + (x + 0.5) * msg_res;
            int mx = static_cast<int>(std::floor((world_x - map_origin_.x) / map_resolution_));

            if (mx < 0 || mx >= map_width_cells_) continue;

            MapCell& cell = main_map_[main_row_offset + mx];

            // 检查时间戳：防止乱序数据把新的覆盖回去了
            if (msg->header.stamp < cell.last_updated) continue;

            // 【核心】：执行拼图
            // 只要这里不重置数据，旧的地图数据就会永远留在 main_map_ 里
            int8_t occ_val = data[msg_row_offset + x];
            stitchMeasurement(cell, occ_val);

            // 更新该栅格的最后刷新时间
            cell.last_updated = msg->header.stamp;
        }
    }
}

void MapManager::updateLoop(const ros::TimerEvent&) {
    if (!robot_pose_received_) return;

    publishFusedMap();
}

void MapManager::publishFusedMap() {
    if (pub_fused_map_.getNumSubscribers() == 0) return;

    nav_msgs::OccupancyGrid fused_map_msg;
    fused_map_msg.header.stamp = ros::Time::now();
    fused_map_msg.header.frame_id = world_frame_;

    fused_map_msg.info.resolution = map_resolution_;
    fused_map_msg.info.width      = map_width_cells_;
    fused_map_msg.info.height     = map_height_cells_;
    fused_map_msg.info.origin.position = map_origin_;
    fused_map_msg.info.origin.orientation.w = 1.0;

    fused_map_msg.data.resize(main_map_.size());
    for (size_t i = 0; i < main_map_.size(); ++i) {
        fused_map_msg.data[i] = main_map_[i].occupancy;
    }

    pub_fused_map_.publish(fused_map_msg);
}

} // namespace fitplane_planner
