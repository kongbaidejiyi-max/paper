#include "fitplane_planner/path_planner.h"
#include <fitplane_planner/GlobalPath.h>   // ★ 新增
#include <algorithm>
#include <chrono>
#include <sstream>

namespace fitplane_planner 
{

namespace {
inline double msSince(std::chrono::steady_clock::time_point start) {
    using namespace std::chrono;
    return duration_cast<duration<double, std::milli>>(steady_clock::now() - start).count();
}
} // namespace

PathPlanner::PathPlanner(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
    // 从参数服务器获取参数
    nh_private_.param("inflation_radius", inflation_radius_, 0.1);
    nh_private_.param("unknown_space_cost", unknown_space_cost_, 1.0);
    nh_private_.param("path_marker_width", path_marker_width_, 0.1);
    nh_private_.param("planning_frequency", planning_frequency_, 1.0); // 默认1Hz
    nh_private_.param("replanning_distance_threshold", replanning_distance_threshold_, 0.5);
    nh_private_.param("rolling_replan_enable", rolling_replan_enable_, true);
    nh_private_.param("rolling_replan_keep_points_behind", rolling_replan_keep_points_behind_, 0);
    nh_private_.param("stitch_replan_enable", stitch_replan_enable_, false);
    nh_private_.param("stitch_replan_trigger_distance", stitch_replan_trigger_distance_m_, 0.2);
    nh_private_.param("stitch_replan_lookahead_distance", stitch_replan_lookahead_distance_m_, 3.0);
    nh_private_.param("astar_raw_marker_enable", astar_raw_marker_enable_, false);
    nh_private_.param("astar_raw_marker_point_size", astar_raw_marker_point_size_, 0.08);
    nh_private_.param("astar_raw_marker_z", astar_raw_marker_z_, 0.03);
    nh_private_.param("astar_raw_marker_max_points", astar_raw_marker_max_points_, 5000);
    nh_private_.param("grp_centerline_marker_enable", grp_centerline_marker_enable_, true);
    nh_private_.param("grp_centerline_marker_width", grp_centerline_marker_width_, 0.05);
    nh_private_.param("grp_centerline_marker_z", grp_centerline_marker_z_, 0.06);
    nh_private_.param("obstacle_distance_cost_weight", obstacle_distance_cost_weight_, 1.0);
    nh_private_.param("turn_penalty_cost_weight", turn_penalty_cost_weight_, 0.5);
    nh_private_.param("goal_weight", goal_weight_, 1.0);
    nh_private_.param("drone_pos_weight", drone_pos_weight_, 0.5);
    nh_private_.param("odom_topic", odom_topic_, std::string("odom"));
    nh_private_.param("world_frame", world_frame_, std::string("map"));
    nh_private_.param("corridor_min_width",  corridor_min_width_,  6.0);  // 整宽，默认6m
    nh_private_.param("corridor_max_width",  corridor_max_width_, 16.0);  // 整宽，默认16m
    nh_private_.param("corridor_margin",     corridor_margin_,     0.5);  // 减去的安全边
    // corridor 可视化参数（/corridor_marker）
    nh_private_.param("corridor_viz_z",                    corridor_viz_z_,                    0.05);
    nh_private_.param("corridor_viz_height",               corridor_viz_height_,               1.0);
    nh_private_.param("corridor_viz_color_mode",           corridor_viz_color_mode_,           std::string("confidence"));
    nh_private_.param("corridor_viz_fill_alpha_min",       corridor_viz_fill_alpha_min_,       0.5);
    nh_private_.param("corridor_viz_fill_alpha_max",       corridor_viz_fill_alpha_max_,       0.35);
    nh_private_.param("corridor_viz_outline_enable",       corridor_viz_outline_enable_,       true);
    nh_private_.param("corridor_viz_outline_width",        corridor_viz_outline_width_,        0.06);
    nh_private_.param("corridor_viz_outline_alpha",        corridor_viz_outline_alpha_,        1.0);
    nh_private_.param("corridor_viz_outline_dashed",       corridor_viz_outline_dashed_,       false);
    nh_private_.param("corridor_viz_dash_length",          corridor_viz_dash_length_,          0.6);
    nh_private_.param("corridor_viz_grid_enable",          corridor_viz_grid_enable_,          true);
    nh_private_.param("corridor_viz_grid_step_m",          corridor_viz_grid_step_m_,          1.0);
    nh_private_.param("corridor_viz_grid_width",           corridor_viz_grid_width_,           0.02);
    nh_private_.param("corridor_viz_grid_alpha",           corridor_viz_grid_alpha_,           0.6);
    nh_private_.param("corridor_viz_centerline_enable",    corridor_viz_centerline_enable_,    false);
    nh_private_.param("corridor_viz_centerline_width",     corridor_viz_centerline_width_,     0.03);
    nh_private_.param("corridor_viz_asym_enable",          corridor_viz_asym_enable_,          true);
    nh_private_.param("corridor_viz_nominal_width",        corridor_viz_nominal_width_,        -1.0);
    nh_private_.param("corridor_viz_known_expand_ratio",   corridor_viz_known_expand_ratio_,   1.2);
    nh_private_.param("corridor_viz_known_unknown_thresh", corridor_viz_known_unknown_thresh_, 0.25);
    nh_private_.param("corridor_viz_clearance_step_m",     corridor_viz_clearance_step_m_,     0.10);
    nh_private_.param("corridor_viz_obstacle_cost_threshold", corridor_viz_obstacle_cost_threshold_, 80.0);
    nh_private_.param("corridor_viz_unknown_is_obstacle",  corridor_viz_unknown_is_obstacle_,  false);
    nh_private_.param("corridor_viz_z_outline_offset",     corridor_viz_z_outline_offset_,     0.01);
    nh_private_.param("corridor_viz_z_grid_offset",        corridor_viz_z_grid_offset_,        0.005);
    nh_private_.param("corridor_viz_z_centerline_offset",  corridor_viz_z_centerline_offset_,  0.02);
    nh_private_.param("corridor_viz_max_segments",         corridor_viz_max_segments_,         300);
    nh_private_.param("corridor_viz_miter_limit",          corridor_viz_miter_limit_,          2.5);
    if (nh_private_.hasParam("clearance_ref_m")) {
        nh_private_.param("clearance_ref_m", clearance_ref_m_, 4.0);  // 归一化参考（米）
    } else {
        // 兼容旧参数名
        nh_private_.param("clearance_ref", clearance_ref_m_, 4.0);
    }

    nh_private_.param("conf_w_trav",         conf_w_trav_,         0.4);
    nh_private_.param("conf_w_slope",        conf_w_slope_,        0.2);
    nh_private_.param("conf_w_unknown",      conf_w_unknown_,      0.2);
    nh_private_.param("conf_w_clearance",    conf_w_clearance_,    0.2);
    nh_private_.param("conf_local_radius_cells", conf_local_radius_cells_, 3);

	    // 稳定栅格参数：防止地图闪烁
	    nh_private_.param("occ_confirm_frames",  occ_confirm_frames_,  2); // 连续2帧占据才变成占据
	    nh_private_.param("free_confirm_frames", free_confirm_frames_, 8); // 连续6帧空闲才变成空闲
        // OccupancyGrid 兼容：很多地图不会严格用 0/100（如 99/120 等），这里用阈值判定障碍
        nh_private_.param("occupied_threshold", occupied_threshold_, 90);

	    nh_private_.param("grp_resample_enable", grp_resample_enable_, true);
	    nh_private_.param("grp_resample_step",   grp_resample_step_m_, 1.0);
	    nh_private_.param("grp_asym_enable",     grp_asym_enable_,     true);

    // 智能重采样参数初始化
    nh_private_.param("safe_resample_enable", safe_resample_enable_, true);
    nh_private_.param("safe_resample_search_radius", safe_resample_search_radius_, 2.0);
    nh_private_.param("adaptive_density_enable", adaptive_density_enable_, true);
    nh_private_.param("high_complexity_threshold", high_complexity_threshold_, 0.7);
    nh_private_.param("low_complexity_threshold", low_complexity_threshold_, 0.3);

    nh_private_.param("ref_path_bias_enable",  ref_path_bias_enable_,  true);
    nh_private_.param("ref_path_bias_weight",  ref_path_bias_weight_,  0.5);  // 0.1~1.0之间自己调
	last_planned_map_version_ = -1;

    // 初始化置信度时间滤波相关成员
    confidence_history_initialized_ = false;
    // 初始化订阅者
    // 使用相对话题名，允许在 launch 中直接 remap：<remap from="fused_map" to="/fused_map"/>
    sub_grid_info_ = nh_.subscribe("fused_map", 1, &PathPlanner::gridInfoCallback, this);
    sub_goal_ = nh_.subscribe("move_base_simple/goal", 1, &PathPlanner::goalCallback, this);
    sub_odom_ = nh_.subscribe(odom_topic_, 1, &PathPlanner::odomCallback, this);
    sub_drone_odom_ = nh_.subscribe("drone/odom", 1, &PathPlanner::droneOdomCallback, this);

    // 发布器初始化：路径、目标与走廊的可视化
    pub_path_ = nh_.advertise<nav_msgs::Path>("plan", 1);
    pub_path_marker_ = nh_.advertise<visualization_msgs::Marker>("plan_marker", 1);
    pub_goal_marker_ = nh_.advertise<visualization_msgs::Marker>("goal_marker", 1);
    // latch: 新订阅者立即收到最近一次 GRP（便于控制/调试）
    pub_grp_ = nh_.advertise<fitplane_planner::GlobalPath>("grp", 1, true);
    // latch: 新订阅者立即收到最近一次 marker（便于 RViz 打开即见）
    pub_astar_raw_marker_ = nh_.advertise<visualization_msgs::Marker>("astar_raw_marker", 1, true);
    pub_grp_centerline_marker_ = nh_.advertise<visualization_msgs::Marker>("grp_centerline_marker", 1, true);
    pub_inflated_costmap_ = nh_.advertise<nav_msgs::OccupancyGrid>("inflated_costmap", 1);
    pub_corridor_marker_ = nh_.advertise<visualization_msgs::Marker>("corridor_marker", 1);

    // 版本初始化
    map_version_ = 0;

    // 设置一个定时器用于规划循环
    planning_timer_ = nh_.createTimer(ros::Duration(1.0 / planning_frequency_), &PathPlanner::planningLoop, this);

    // 用当前时间初始化随机数生成器
    random_generator_.seed(std::chrono::system_clock::now().time_since_epoch().count());

    ROS_INFO("Path planner initialized.");
}


PathPlanner::~PathPlanner() {}

void PathPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_robot_pose_ = msg->pose.pose;
    //将其设置为0
    // current_robot_pose_.position.x = 0;
    // current_robot_pose_.position.y = 0;
    // current_robot_pose_.position.z = 0;
    // //姿态设置为0
    // current_robot_pose_.orientation.x = 0;
    // current_robot_pose_.orientation.y = 0;
    // current_robot_pose_.orientation.z = 0;
    // current_robot_pose_.orientation.w = 0;
    pose_received_ = true;
    ROS_INFO_ONCE("Odometry received, planner is ready to receive goals.");
}



void PathPlanner::droneOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_drone_pose_ = msg->pose.pose;
    drone_pose_received_ = true;
}



void PathPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_goal_ = *msg;
    goal_received_ = true;
    is_planning_active_ = true;
    new_goal_trigger_ = true; // 触发新目标处理逻辑

    // 关键：清除上一条路径，以确保对新目标立即进行重规划
    last_published_path_.poses.clear();
    stitch_replan_baseline_valid_ = false;

    ROS_INFO("New goal received! Forcing an immediate replan.");
}



void PathPlanner::planningLoop(const ros::TimerEvent& event) {
    // 周期性地运行规划循环
    runPlanningCycle();
}

void PathPlanner::runPlanningCycle() {
    const auto t_cycle_start = std::chrono::steady_clock::now();
    double ms_oldprep = 0.0;
    double ms_old_to_idx = 0.0;
    double ms_old_find_unsafe = 0.0;
    double ms_astar_full = 0.0;
    double ms_astar_suffix = 0.0;
    double ms_remove_loops = 0.0;
    double ms_resample = 0.0;
    double ms_simplify = 0.0;
    double ms_publish = 0.0;

    if (!is_planning_active_ || !goal_received_) {
        return;
    }

    if (!map_received_ || !pose_received_) {
        ROS_WARN_THROTTLE(3.0, "Planner is active but waiting for map or odom data.");
        return;
    }

    // --- 新目标处理逻辑 ---
    if (new_goal_trigger_) {
        new_goal_trigger_ = false; // 重置触发器
        is_searching_for_valid_goal_ = false; // 停止任何正在进行的旧搜索

        Eigen::Vector2i goal_idx = worldToGrid(current_goal_.pose.position);
        if (getCost(goal_idx) < 0) {
            // 目标无效，启动搜索模式
            is_searching_for_valid_goal_ = true;
            search_start_time_ = ros::Time::now();
            search_center_ = goal_idx;
            search_radius_ = grid_info_->info.resolution * 2; // 初始搜索半径设为2个栅格的宽度
            ROS_WARN("Original goal is invalid. Starting random search for a valid nearby goal (timeout 5s)...");
        }
    }

    // --- 增量式搜索逻辑 ---
    if (is_searching_for_valid_goal_) {
        // 检查超时
        if (ros::Time::now() - search_start_time_ > ros::Duration(5.0)) {
            ROS_ERROR("Failed to find a valid goal within 5 seconds. Aborting planning.");
            ROS_WARN("Timing: abort search (spent %.1f ms in this cycle).", msSince(t_cycle_start));
            is_planning_active_ = false;
            is_searching_for_valid_goal_ = false;
            return;
        }

        bool found_valid_in_chunk = false;
        // 在每个规划周期内尝试一定次数的随机采样
        int attempts_per_chunk = 100; 
        
        std::uniform_real_distribution<double> angle_dist(0.0, 2.0 * M_PI);
        std::uniform_real_distribution<double> radius_dist(0.0, search_radius_);

        for (int i = 0; i < attempts_per_chunk; ++i) {
            double angle = angle_dist(random_generator_);
            double radius = radius_dist(random_generator_);

            // 将极坐标转换为栅格偏移量
            int dx = static_cast<int>(radius / grid_info_->info.resolution * std::cos(angle));
            int dy = static_cast<int>(radius / grid_info_->info.resolution * std::sin(angle));
            
            Eigen::Vector2i current_check_pos = search_center_ + Eigen::Vector2i(dx, dy);

            if (current_check_pos.x() >= 0 && current_check_pos.x() < grid_info_->info.width &&
                current_check_pos.y() >= 0 && current_check_pos.y() < grid_info_->info.height &&
                getCost(current_check_pos) >= 0) {
                
                ROS_INFO("Found a valid alternative goal at (%d, %d).", current_check_pos.x(), current_check_pos.y());
                current_goal_.pose.position = gridToWorld(current_check_pos);
                is_searching_for_valid_goal_ = false;
                found_valid_in_chunk = true;
                break; 
            }
        }

        if (!found_valid_in_chunk) {
            // 如果没找到，则增大搜索半径，等待下个周期
            search_radius_ *= 1.2; 
            ROS_INFO_THROTTLE(1.0, "Searching for a valid goal... Current search radius: %.2f m", search_radius_);
            ROS_INFO_THROTTLE(1.0, "Timing: searching goal (spent %.1f ms in this cycle).", msSince(t_cycle_start));
            return; 
        }
        // 如果找到了，将重置 last_published_path_ 以强制进行一次全新的规划
        last_published_path_.poses.clear();
    }

    // --- 到达目标判断（必须在“复用旧路径 return”之前，否则会一直复用而无法退出） ---
    const double dist_to_goal = std::hypot(current_robot_pose_.position.x - current_goal_.pose.position.x,
                                          current_robot_pose_.position.y - current_goal_.pose.position.y);
    if (dist_to_goal < 0.2) {
        ROS_INFO("Goal reached! Deactivating planning.");
        is_planning_active_ = false;
        last_published_path_.poses.clear();
        nav_msgs::Path empty_path;
        empty_path.header.stamp = ros::Time::now();
        empty_path.header.frame_id = world_frame_;
        pub_path_.publish(empty_path);
        publishPathMarker(empty_path);
        publishGoalMarker(true); // 到达后删除目标点标记
        return;
    }

    // --- 路径保持逻辑 ---
    // 如果存在一条旧路径，并且它仍然有效，且机器人离它足够近
    // if (!last_published_path_.poses.empty() && isPathStillValid() && distanceToPath() < replanning_distance_threshold_) {
    //     ROS_INFO_THROTTLE(5.0, "Current path is still valid and robot is close. Skipping replanning.");
    //     // (可选) 可以在这里重新发布旧路径，以防某些订阅者错过了消息
    //     pub_path_.publish(last_published_path_);
    //     publishPathMarker(last_published_path_);
    //     publishGoalMarker();
    //     return;
    // }
    // --- 路径保持逻辑（加入地图版本 & 走廊安全）---
	/* 暂时注释掉
	if (!last_published_path_.poses.empty()) {
		double dist_to_path = distanceToPath();
		double safety_margin_m = corridor_margin_ + 0.3;  // 比 corridor_margin_ 稍保守

		bool path_valid = isPathStillValid();
		bool corridor_safe = isCorridorStillSafe(safety_margin_m);

		// 地图即使更新了，只要旧路径在新地图下仍然安全，就允许复用（避免轻微地图抖动导致路径频繁变化）
		if (
			path_valid &&                                   // 栅格上仍可行
			corridor_safe &&                               // 走廊清空度足够
			dist_to_path < replanning_distance_threshold_) // 机器人离路径不远
			{
                // --- 新增：短段拼接式重规划（按里程触发，且尽量贴着旧路径）---
                if (stitch_replan_enable_) {
                    if (!stitch_replan_baseline_valid_) {
                        last_stitch_replan_position_ = current_robot_pose_.position;
                        stitch_replan_baseline_valid_ = true;
                    }

                    const double dx = current_robot_pose_.position.x - last_stitch_replan_position_.x;
                    const double dy = current_robot_pose_.position.y - last_stitch_replan_position_.y;
                    const double moved_m = std::hypot(dx, dy);

                    const double trigger_m = stitch_replan_trigger_distance_m_;
                    const bool trigger_now = (trigger_m <= 0.0) || (moved_m >= trigger_m);

                    if (trigger_now) {
                        const int k = closestIndexOnPath(last_published_path_, current_robot_pose_.position);
                        if (k >= 0 && last_published_path_.poses.size() >= 2) {
                            const double lookahead_m = std::max(0.0, stitch_replan_lookahead_distance_m_);

                            // 1) 选择拼接点：从 k 开始沿旧路径累计距离，找到前方约 lookahead_m 的点
                            int j = k;
                            double acc = 0.0;
                            for (size_t i = static_cast<size_t>(k); i + 1 < last_published_path_.poses.size(); ++i) {
                                const auto& p1 = last_published_path_.poses[i].pose.position;
                                const auto& p2 = last_published_path_.poses[i + 1].pose.position;
                                acc += std::hypot(p2.x - p1.x, p2.y - p1.y);
                                if (acc >= lookahead_m) {
                                    j = static_cast<int>(i + 1);
                                    break;
                                }
                                j = static_cast<int>(i + 1);
                            }
                            if (j <= k && (k + 1) < static_cast<int>(last_published_path_.poses.size())) {
                                j = k + 1;
                            }

                            // 2) 构建旧路径索引 & bias 距离场
                            std::vector<Eigen::Vector2i> old_idx = pathMsgToIndices(last_published_path_);
                            if (!old_idx.empty()) {
                                if (j >= static_cast<int>(old_idx.size())) j = static_cast<int>(old_idx.size()) - 1;
                                if (j < 0) j = 0;

                                const Eigen::Vector2i stitch_goal = old_idx[static_cast<size_t>(j)];
                                Eigen::Vector2i start_idx = worldToGrid(current_robot_pose_.position);

                                // 如果当前位置落在不可用格子上（例如障碍/越界），回退到旧路径最近点
                                if (getCost(start_idx) < 0.0) {
                                    start_idx = old_idx[static_cast<size_t>(k)];
                                }

                                // 仅用于本次短段规划的 soft bias：鼓励贴近旧路径
                                buildDistanceToOldPath(old_idx);

                                // 3) 构造 blocked：禁止回到"已走过"的旧路径前缀，避免短段规划绕回头
                                const int W = grid_info_->info.width;
                                const int H = grid_info_->info.height;
                                std::vector<bool> blocked(W * H, false);
                                const int keep_behind = std::max(0, rolling_replan_keep_points_behind_);
                                const int block_upto = std::max(0, k - keep_behind);
                                for (int i = 0; i < block_upto && i < static_cast<int>(old_idx.size()); ++i) {
                                    const auto& p = old_idx[static_cast<size_t>(i)];
                                    if (p == start_idx || p == stitch_goal) continue;
                                    if (p.x() < 0 || p.x() >= W || p.y() < 0 || p.y() >= H) continue;
                                    blocked[p.y() * W + p.x()] = true;
                                }

                                // 4) 短段 A*：start -> stitch_goal
                                std::vector<Eigen::Vector2i> prefix;
                                if (start_idx == stitch_goal) {
                                    prefix.push_back(start_idx);
                                } else {
                                    prefix = findPath(start_idx, stitch_goal, &blocked);
                                }

                                if (!prefix.empty()) {
                                    // 5) 拼接：prefix + old[j+1..end]
                                    std::vector<Eigen::Vector2i> stitched = prefix;
                                    if (stitched.back().x() != stitch_goal.x() ||
                                        stitched.back().y() != stitch_goal.y()) {
                                        stitched.push_back(stitch_goal);
                                    }
                                    for (size_t t = static_cast<size_t>(j + 1); t < old_idx.size(); ++t) {
                                        stitched.push_back(old_idx[t]);
                                    }

                                    // 用旧路径末端作为"当前规划目标"，避免探索模式下把最后点改成 map 外的最终目标
                                    publishPath(stitched, old_idx.back());

                                    last_stitch_replan_position_ = current_robot_pose_.position;
                                    stitch_replan_baseline_valid_ = true;

                                    publishGoalMarker();
                                    return;
                                }
                            }
                        }
                    }
                }

				ROS_WARN_THROTTLE(1.0,
					"[REUSING_PATH] Reusing existing path (map_version=%d, dist_to_path=%.2f m).",
					map_version_, dist_to_path);

                // 复用时也做"滚动裁剪"，确保 /plan 的起点随机器人前进；
                // 并通过 publishPath() 统一刷新 header/yaw，并把 last_planned_map_version_ 对齐到当前地图版本。
                std::vector<Eigen::Vector2i> idx = pathMsgToIndices(last_published_path_);
                if (!idx.empty() && rolling_replan_enable_) {
                    int k = closestIndexOnPath(last_published_path_, current_robot_pose_.position);
                    if (k >= 0) {
                        const int keep_behind = std::max(0, rolling_replan_keep_points_behind_);
                        int trim_to = k - keep_behind;
                        if (trim_to < 0) trim_to = 0;
                        if (trim_to > static_cast<int>(idx.size()) - 1) {
                            trim_to = static_cast<int>(idx.size()) - 1; // 至少保留一个点
                        }
                        if (trim_to > 0) {
                            idx.erase(idx.begin(), idx.begin() + trim_to);
                        }
                    }
                }
                if (!idx.empty()) {
                    publishPath(idx, idx.back());
                } else {
                    pub_path_.publish(last_published_path_);
                    publishPathMarker(last_published_path_);
                    last_planned_map_version_ = static_cast<int>(map_version_);
                    publishGRP(last_published_path_);
                }
                publishGoalMarker();
				return;
			}
	}
	*/


    // --- 正常规划逻辑 ---
    
    // Eigen::Vector2i start_idx = worldToGrid(current_robot_pose_.position);
    // Eigen::Vector2i goal_idx = worldToGrid(current_goal_.pose.position);
	// === 起点：优先使用“当前机器人在旧路径上的投影点” ===
	Eigen::Vector2i start_idx;
	if (!last_published_path_.poses.empty()) {
		int k = closestIndexOnPath(last_published_path_, current_robot_pose_.position);
		if (k >= 0) {
			start_idx = worldToGrid(last_published_path_.poses[k].pose.position);
		} else {
			start_idx = worldToGrid(current_robot_pose_.position);
		}
	} else {
		start_idx = worldToGrid(current_robot_pose_.position);
	}

	Eigen::Vector2i goal_idx = worldToGrid(current_goal_.pose.position);

    // 检查最终目标是否在当前地图范围内
    bool is_goal_in_map = (goal_idx.x() >= 0 && goal_idx.x() < grid_info_->info.width &&
                           goal_idx.y() >= 0 && goal_idx.y() < grid_info_->info.height);

    Eigen::Vector2i actual_planning_goal;
    if (is_goal_in_map) {
        // 目标在地图内，正常规划
        actual_planning_goal = goal_idx;
        ROS_INFO_ONCE("Goal is inside the map. Standard planning mode.");
    } else {
        // 目标在地图外，进入探索模式
        if (!drone_pose_received_) {
            ROS_WARN_THROTTLE(5.0, "Drone pose not received yet. Using goal-only exploration.");
        }
        ROS_INFO_THROTTLE(5.0, "Goal is outside the map. Finding an exploration proxy goal.");
        actual_planning_goal = findExplorationGoal();
    }

    // 检查目标点是否有效，如果无效则寻找替代点（此逻辑对标准目标和探索目标都适用）
    Eigen::Vector2i original_goal_idx = actual_planning_goal;
     if (!is_searching_for_valid_goal_ && getCost(actual_planning_goal) < 0) {
        ROS_WARN("Planning goal at (%d, %d) is invalid. Starting search for a new goal.", actual_planning_goal.x(), actual_planning_goal.y());
        is_searching_for_valid_goal_ = true;
        search_start_time_ = ros::Time::now();
        search_center_ = actual_planning_goal; 
        search_radius_ = grid_info_->info.resolution * 2; 
        return; 
    }

    ROS_INFO_THROTTLE(2.0, "Replanning from (%d, %d) to (%d, %d)",
        start_idx.x(), start_idx.y(),
        actual_planning_goal.x(), actual_planning_goal.y());

    // ============================================================
    // 修正：考虑小车当前位置，只保留从小车位置往后的安全部分
    // ============================================================
    std::vector<Eigen::Vector2i> path_indices;

    // 额外的安全裕度（比 corridor_margin_ 再保守一点）
    const double safety_margin_m = corridor_margin_ + 0.3;

    if (!last_published_path_.poses.empty()) {
        // 1) 把旧路径转成栅格索引
        const auto t_oldprep = std::chrono::steady_clock::now();
        const auto t_old_to_idx = std::chrono::steady_clock::now();
        std::vector<Eigen::Vector2i> old_idx = pathMsgToIndices(last_published_path_);
        ms_old_to_idx = msSince(t_old_to_idx);

        // 1.5) 找到小车在旧路径上的位置 k
        int k = closestIndexOnPath(last_published_path_, current_robot_pose_.position);
        if (k < 0) k = 0;
        if (static_cast<size_t>(k) >= old_idx.size()) k = static_cast<int>(old_idx.size()) - 1;

        ROS_INFO("Robot closest index on old path: %d (path size=%zu)", k, old_idx.size());

        // 1.6) 启用"贴近旧路径"的 soft bias（只对 k 之后的路径段有效）
        std::vector<Eigen::Vector2i> old_idx_suffix(old_idx.begin() + k, old_idx.end());
        buildDistanceToOldPath(old_idx_suffix);

        // 2) 从 k 开始往后找第一个"不安全"的点（越界 / 压障碍 / clearance 太小）
        const auto t_old_find_unsafe = std::chrono::steady_clock::now();
        size_t cut_relative = findFirstUnsafeIndex(old_idx_suffix, safety_margin_m);
        size_t cut = k + cut_relative;  // 转换为在完整 old_idx 中的索引
        ms_old_find_unsafe = msSince(t_old_find_unsafe);
        ms_oldprep = msSince(t_oldprep);

        ROS_INFO("First unsafe index: %zu (relative to k: %zu)", cut, cut_relative);

        if (cut_relative == old_idx_suffix.size()) {
            // 情况 A：从 k 往后的路径都安全
            // 直接裁掉 k 之前的部分，保留 [k, end)
            std::vector<Eigen::Vector2i> trimmed(old_idx.begin() + k, old_idx.end());
            path_indices = trimmed;
            ROS_INFO_THROTTLE(3.0,
                "Path from robot position is fully safe, trimmed to %zu points.", path_indices.size());
        } else {
            // 情况 B：从 k 往后在 cut 处变得不安全
            //   -> 保留 [k, cut) 作为前缀
            //   -> 从 cut 重新规划到目标

            if (cut <= static_cast<size_t>(k)) {
                // cut 在 k 之前或等于 k，没有安全前缀可用，整条重新规划
                ROS_WARN("No safe prefix from robot position, planning from scratch.");
                const auto t_full = std::chrono::steady_clock::now();
                path_indices = findPath(start_idx, actual_planning_goal);
                ms_astar_full = msSince(t_full);
            } else {
                // 有安全前缀 [k, cut)
                std::vector<Eigen::Vector2i> prefix(old_idx.begin() + k,
                                                    old_idx.begin() + cut);
                Eigen::Vector2i stitch_start = prefix.back();

                ROS_INFO("Using safe prefix [%d, %zu), size=%zu", k, cut, prefix.size());

                // --- 构造 blocked_cells 掩码：前缀中的格子（除了 stitch_start）全部禁止 ---
                const int W = grid_info_->info.width;
                const int H = grid_info_->info.height;
                std::vector<bool> blocked(W * H, false);

                for (size_t i = 0; i + 1 < prefix.size(); ++i) { // 注意 +1，保留最后一个点
                    const auto& p = prefix[i];
                    if (p.x() < 0 || p.x() >= W || p.y() < 0 || p.y() >= H) continue;
                    int idx = p.y() * W + p.x();
                    blocked[idx] = true;
                }

                // 后缀规划时不再用旧路径 bias，免得吸到不安全段附近
                distance_to_old_path_.clear();

                // --- 从 stitch_start 到 actual_planning_goal 规划后缀，禁止回到 blocked 区 ---
                const auto t_suffix = std::chrono::steady_clock::now();
                std::vector<Eigen::Vector2i> suffix =
                    findPath(stitch_start, actual_planning_goal, &blocked);
                ms_astar_suffix = msSince(t_suffix);

                if (!suffix.empty()) {
                    // suffix[0] == stitch_start，与前缀最后一个点重复，删掉
                    suffix.erase(suffix.begin());

                    path_indices = prefix;
                    path_indices.insert(path_indices.end(), suffix.begin(), suffix.end());

                    ROS_INFO("Stitched new suffix: prefix=%zu, suffix=%zu, total=%zu",
                             prefix.size(), suffix.size(), path_indices.size());
                } else {
                    // 如果从拼接点到目标的规划失败，再退回"整条重新规划"
                    ROS_WARN("Failed to find suffix path from stitch_start, "
                             "falling back to full replan.");
                    const auto t_full = std::chrono::steady_clock::now();
                    path_indices = findPath(start_idx, actual_planning_goal);
                    ms_astar_full = msSince(t_full);
                }
            }
        }
    } else {
        ROS_WARN("No old path available, planning from scratch.");
        // 没有旧路径可用，直接从头规划
        const auto t_full = std::chrono::steady_clock::now();
        path_indices = findPath(start_idx, actual_planning_goal);
        ms_astar_full = msSince(t_full);
    }

    // ============================================================
    // 处理规划结果
    // ============================================================
	    if (path_indices.empty()) {
	    // A* 规划失败，说明当前目标点（即使不在障碍物里）也是不可达的
	    ROS_WARN("No path found to the current planning goal. It might be in an unreachable area. "
	    "Starting search for a new goal.");
	    is_searching_for_valid_goal_ = true;
	    search_start_time_ = ros::Time::now();
	    search_center_ = actual_planning_goal; // 围绕这个不可达的点开始搜索
	    search_radius_ = grid_info_->info.resolution * 2; // 重置搜索半径
	    return; // 结束本次循环，下一轮将进入搜索逻辑
		    } else {
		    // 0) 可视化：A* 原始每步格子点（后处理前）
		    publishAstarRawMarker(path_indices);
		    // 1) 去环
		    // TEMP: 暂时关闭后处理（去环、简化、重采样）
		    auto no_loop = path_indices;
		    // const auto t_remove = std::chrono::steady_clock::now();
		    // auto no_loop = removePathLoops(path_indices);
		    // ms_remove_loops = msSince(t_remove);

	    // 2) 先做一次几何简化（基于 isLineFree，可直接消除很多"网格折线"的拐点）
	    auto simplified_base = no_loop;
	    // if (no_loop.size() > 2) {
	    //     const auto t_simplify = std::chrono::steady_clock::now();
	    //     auto tmp = simplifyPath(no_loop);
	    //     ms_simplify = msSince(t_simplify);
	    //     if (tmp.size() >= 2) {
	    //         simplified_base = std::move(tmp);
	    //     }
	    // }

	    // 3) 再重采样（给跟踪器更密的点，拐弯更"圆滑"）
	    auto dense_path = simplified_base;
	    // if (safe_resample_enable_ && grp_resample_step_m_ > 1e-3) {
	    //     ROS_INFO("Path postprocess: simplify+intelligent_resample (%zu -> ... step %.2fm)",
	    //              simplified_base.size(), grp_resample_step_m_);
	    //     const auto t_resample = std::chrono::steady_clock::now();
	    //     dense_path = intelligentSafeResample(simplified_base, grp_resample_step_m_);
	    //     ms_resample = msSince(t_resample);
	    // } else if (grp_resample_enable_ && grp_resample_step_m_ > 1e-3) {
	    //     const auto t_resample = std::chrono::steady_clock::now();
	    //     dense_path = resamplePath(simplified_base, grp_resample_step_m_);
	    //     ms_resample = msSince(t_resample);
	    // }

	    ROS_INFO_THROTTLE(2.0,
	        "Path found: raw=%zu, no_loop=%zu, simplified=%zu, dense=%zu.",
	        path_indices.size(), no_loop.size(), simplified_base.size(), dense_path.size());

		    const auto t_publish = std::chrono::steady_clock::now();
		    publishPath(dense_path, actual_planning_goal);
		    ms_publish = msSince(t_publish);
            if (stitch_replan_enable_) {
                last_stitch_replan_position_ = current_robot_pose_.position;
                stitch_replan_baseline_valid_ = true;
            }

    const double ms_total = msSince(t_cycle_start);
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss.precision(1);
    oss << "Timing(ms): total=" << ms_total
        << " oldprep=" << ms_oldprep
        << " old_to_idx=" << ms_old_to_idx
        << " old_find_unsafe=" << ms_old_find_unsafe
        << " astar_full=" << ms_astar_full
        << " astar_suffix=" << ms_astar_suffix
        << " remove_loops=" << ms_remove_loops
        << " resample=" << ms_resample
        << " simplify=" << ms_simplify
        << " publish=" << ms_publish;
    ROS_INFO_STREAM_THROTTLE(1.0, oss.str());
    }

    // 无论规划是否成功，都更新目标点的可视化
    publishGoalMarker();
    }

int PathPlanner::closestIndexOnPath(const nav_msgs::Path& path,
	const geometry_msgs::Point& p_world) const
{
	if (path.poses.empty()) return -1;

	double best_dist2 = std::numeric_limits<double>::max();
	int best_idx = -1;

	for (size_t i = 0; i < path.poses.size(); ++i) {
	const auto& q = path.poses[i].pose.position;
	double dx = q.x - p_world.x;
	double dy = q.y - p_world.y;
	double d2 = dx * dx + dy * dy;
	if (d2 < best_dist2) {
	best_dist2 = d2;
	best_idx = static_cast<int>(i);
	}
	}
	return best_idx;
}



bool PathPlanner::isPathStillValid() {
    for (const auto& pose_stamped : last_published_path_.poses) {
        Eigen::Vector2i grid_pos = worldToGrid(pose_stamped.pose.position);
        if (getCost(grid_pos) < 0) {
            ROS_WARN("Path is no longer valid, an obstacle was found on the path.");
            return false;
        }
    }
    return true;
}



double PathPlanner::distanceToPath() {
    double min_dist_sq = std::numeric_limits<double>::max();
    const auto& robot_pos = current_robot_pose_.position;

    if (last_published_path_.poses.empty()) {
        return min_dist_sq;
    }

    if (last_published_path_.poses.size() == 1) {
        const auto& p = last_published_path_.poses.front().pose.position;
        return std::hypot(robot_pos.x - p.x, robot_pos.y - p.y);
    }

    for (size_t i = 0; i < last_published_path_.poses.size() - 1; ++i) {
        const auto& p1 = last_published_path_.poses[i].pose.position;
        const auto& p2 = last_published_path_.poses[i+1].pose.position;
        
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;

        if (dx == 0 && dy == 0) { // 如果线段长度为0
            double dist_sq = (robot_pos.x - p1.x)*(robot_pos.x - p1.x) + (robot_pos.y - p1.y)*(robot_pos.y - p1.y);
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
            }
            continue;
        }

        // 计算点到线段的投影
        double t = ((robot_pos.x - p1.x) * dx + (robot_pos.y - p1.y) * dy) / (dx*dx + dy*dy);
        
        double closest_x, closest_y;
        if (t < 0.0) {
            closest_x = p1.x;
            closest_y = p1.y;
        } else if (t > 1.0) {
            closest_x = p2.x;
            closest_y = p2.y;
        } else {
            closest_x = p1.x + t * dx;
            closest_y = p1.y + t * dy;
        }

        double dist_sq = (robot_pos.x - closest_x)*(robot_pos.x - closest_x) + (robot_pos.y - closest_y)*(robot_pos.y - closest_y);
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
        }
    }
    
    return std::sqrt(min_dist_sq);
}



Eigen::Vector2i PathPlanner::findExplorationGoal() {
    Eigen::Vector2i best_goal(-1, -1);
    double max_score = -1.0;

    const Eigen::Vector2i final_goal_idx = worldToGrid(current_goal_.pose.position);
    const Eigen::Vector2i drone_pos_idx = worldToGrid(current_drone_pose_.position);

    // 遍历地图的所有边界点
    for (int x = 0; x < grid_info_->info.width; ++x) {
        for (int y = 0; y < grid_info_->info.height; ++y) {
            // 只考虑边界上的点
            if (x == 0 || x == grid_info_->info.width - 1 || y == 0 || y == grid_info_->info.height - 1) {
                Eigen::Vector2i current_pos(x, y);
                // 检查该点是否为可达区域
                if (getCost(current_pos) >= 0) {
                    double dist_to_final_goal_sq = (current_pos.x() - final_goal_idx.x()) * (current_pos.x() - final_goal_idx.x()) +
                                                   (current_pos.y() - final_goal_idx.y()) * (current_pos.y() - final_goal_idx.y());
                    
                    // 避免除以0
                    double goal_score = (dist_to_final_goal_sq > 1e-6) ? 1.0 / std::sqrt(dist_to_final_goal_sq) : 1e6;

                    double drone_score = 0.0;
                    if (drone_pose_received_) {
                        double dist_to_drone_sq = (current_pos.x() - drone_pos_idx.x()) * (current_pos.x() - drone_pos_idx.x()) +
                                                  (current_pos.y() - drone_pos_idx.y()) * (current_pos.y() - drone_pos_idx.y());
                        drone_score = (dist_to_drone_sq > 1e-6) ? 1.0 / std::sqrt(dist_to_drone_sq) : 1e6;
                    }
                    
                    double current_score = goal_weight_ * goal_score + drone_pos_weight_ * drone_score;
                    
                    if (current_score > max_score) {
                        max_score = current_score;
                        best_goal = current_pos;
                    }
                }
            }
        }
    }

    if (best_goal.x() == -1) {
        // 如果没找到任何有效的边界点（一个不太可能发生的极端情况），就返回地图中心
        ROS_WARN("Could not find any valid exploration goal on the map boundary. Defaulting to map center.");
        return Eigen::Vector2i(grid_info_->info.width / 2, grid_info_->info.height / 2);
    }

    return best_goal;
}

} // namespace fitplane_planner
