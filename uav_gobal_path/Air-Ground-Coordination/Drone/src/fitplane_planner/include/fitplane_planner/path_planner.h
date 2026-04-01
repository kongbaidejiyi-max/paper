#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <ros/ros.h>
#include <vector>
#include <queue>
#include <cmath>
#include <Eigen/Dense>
#include <random>
#include <unordered_map>
#include <chrono>

#include <fitplane/GridPlaneInfo.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>

namespace fitplane_planner {

// // A* 算法的节点结构
// struct Node {
//     Eigen::Vector2i position; // 栅格坐标
//     double g_cost; // 从起点到当前节点的代价
//     double h_cost; // 从当前节点到终点的启发式代价
//     double f_cost; // g_cost + h_cost
//     Node* parent;

//     Node(Eigen::Vector2i pos, double g, double h, Node* p)
//         : position(pos), g_cost(g), h_cost(h), f_cost(g + h), parent(p) {}

//     // 用于优先队列的比较函数
//     struct Compare {
//         bool operator()(const Node* a, const Node* b) const {
//             return a->f_cost > b->f_cost;
//         }
//     };
// };
struct GridNode {
    double g = std::numeric_limits<double>::infinity();
    double h = 0.0;
    int    parent = -1;
    bool   in_open = false;
    bool   closed  = false;
};

struct NodeCompare {
    const std::vector<GridNode>* nodes;
    explicit NodeCompare(const std::vector<GridNode>* n = nullptr) : nodes(n) {}
    bool operator()(int a, int b) const {
        const auto& na = (*nodes)[a];
        const auto& nb = (*nodes)[b];
        return (na.g + na.h) > (nb.g + nb.h);  // 小 f 优先
    }
};

// 带记忆的稳定栅格结构，用于防止地图闪烁
struct StableCell {
    int8_t  state;    // -1: unknown, 0: free, 100: occupied —— 给规划用的稳定占据
    uint8_t occ_cnt;  // 连续观测为 100 的次数
    uint8_t free_cnt; // 连续观测为   0 的次数
};
class PathPlanner {
public:
    PathPlanner(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    ~PathPlanner();
private:
    ros::Publisher pub_grp_;
    ros::Publisher pub_astar_raw_marker_;
    ros::Publisher pub_grp_centerline_marker_;

    // marker 可视化：A* 原始每步格子点（用于调试）
    bool   astar_raw_marker_enable_;
    double astar_raw_marker_point_size_;
    double astar_raw_marker_z_;
    int    astar_raw_marker_max_points_;

    // marker 可视化：/grp 中心线点列（直接从 grp.poses 画）
    bool   grp_centerline_marker_enable_;
    double grp_centerline_marker_width_;
    double grp_centerline_marker_z_;

    // --- 新增：GRP 相关参数 ---
    double corridor_min_width_;      // m，走廊最小整宽 -> 半宽为其一半
    double corridor_max_width_;      // m，走廊最大整宽
    double corridor_margin_;         // m，走廊留边（对距障碍的保守扣减）
    double clearance_ref_m_;         // m，置信度中清空度的归一化参考

    // --- 可视化参数：corridor_marker ---
    double corridor_viz_z_;                    // m，走廊底面离地高度（避免与 costmap z-fighting）
    double corridor_viz_height_;               // m，走廊可视化高度（形成 3D 管道）
    std::string corridor_viz_color_mode_;      // "confidence" | "time"

    double corridor_viz_fill_alpha_min_;       // 走廊填充最小透明度（低值）
    double corridor_viz_fill_alpha_max_;       // 走廊填充最大透明度（高值）

    bool   corridor_viz_outline_enable_;       // 是否绘制走廊边界线
    double corridor_viz_outline_width_;        // m，走廊边界线宽
    double corridor_viz_outline_alpha_;        // 走廊边界线透明度（建议 1.0）
    bool   corridor_viz_outline_dashed_;       // 是否虚线
    double corridor_viz_dash_length_;          // m，虚线 dash 长度（gap 同长）

    bool   corridor_viz_grid_enable_;          // 是否绘制内部网格线
    double corridor_viz_grid_step_m_;          // m，沿走廊方向的网格步长
    double corridor_viz_grid_width_;           // m，网格线宽
    double corridor_viz_grid_alpha_;           // 网格线透明度

    bool   corridor_viz_centerline_enable_;    // 是否绘制中心线
    double corridor_viz_centerline_width_;     // m，中心线线宽

    bool   corridor_viz_asym_enable_;          // 是否启用“非对称走廊”（靠障一侧缩、远离侧尽量保持/扩展）
    double corridor_viz_nominal_width_;        // m，走廊期望整宽（<=0 则使用 corridor_min_width_）
    double corridor_viz_known_expand_ratio_;   // 已知区域走廊适度扩张倍率（>1 才生效）
    double corridor_viz_known_unknown_thresh_; // unknown_ratio <= 阈值 视为“已知区域”
    double corridor_viz_clearance_step_m_;     // m，单侧清空度采样步长
    double corridor_viz_obstacle_cost_threshold_; // >阈值视为障碍/不可进入（用于膨胀区）
    bool   corridor_viz_unknown_is_obstacle_;  // 是否将 unknown 视为障碍（更保守）

    double corridor_viz_z_outline_offset_;     // m，边界线相对 fill 的高度偏移
    double corridor_viz_z_grid_offset_;        // m，网格线相对 fill 的高度偏移
    double corridor_viz_z_centerline_offset_;  // m，中心线相对 fill 的高度偏移
    int    corridor_viz_max_segments_;         // 限制可视化段数，避免 marker 过大
    double corridor_viz_miter_limit_;          // miter 上限（倍数）：用于 2D ribbon/拐角处理（备用）

    // 置信度权重
    double conf_w_trav_;             // 可通行（低风险）权重
    double conf_w_slope_;            // 坡度权重
    double conf_w_unknown_;          // 未知率权重
    double conf_w_clearance_;        // 清空度权重
    int    conf_local_radius_cells_; // 统计未知率的圆邻域半径（格）

    // 路径重采样（便于下游）
    bool   grp_resample_enable_;
    double grp_resample_step_m_;

    // GRP 走廊表达增强（不改 msg 的前提下）：
    // 通过“横向偏移 poses.centerline + corridor_half_width=平均半宽”的方式，等价表达左右非对称走廊。
    bool   grp_asym_enable_;

    // 智能重采样相关参数
    bool   safe_resample_enable_;
    double safe_resample_search_radius_;
    bool   adaptive_density_enable_;
    double high_complexity_threshold_;
    double low_complexity_threshold_;

    // 地图/路径版本管理
    uint32_t map_version_;
    ros::Time last_map_update_time_;

    // --- 新增：方法 ---
    void publishGRP(const nav_msgs::Path& path_msg);
    void computeCorridorAndConfidence(const std::vector<Eigen::Vector2i>& path_indices,
                                    std::vector<float>& half_width,
                                    std::vector<float>& confidence) const;
    float computeUnknownRatio(const Eigen::Vector2i& center, int radius_cells) const;
    std::vector<Eigen::Vector2i> resamplePath(const std::vector<Eigen::Vector2i>& path_indices,
                                            double step_m) const;

    // 智能安全重采样函数
    std::vector<Eigen::Vector2i> intelligentSafeResample(
        const std::vector<Eigen::Vector2i>& path_indices,
        double base_step_m) const;

    // 安全检查相关函数
    bool isPositionSafe(const Eigen::Vector2i& pos, double safety_margin = 0.0) const;
    bool isCorridorSafeAtPosition(const Eigen::Vector2i& pos, double half_width) const;
    bool hasObstacleInCorridor(const Eigen::Vector2i& center_pos, double half_width) const;

    // 局部重规划相关函数
    std::vector<Eigen::Vector2i> localReplan(const Eigen::Vector2i& start_pos,
                                            const Eigen::Vector2i& end_pos) const;
    std::vector<bool> createBlockedMask(const Eigen::Vector2i& start,
                                       const Eigen::Vector2i& end) const;
    Eigen::Vector2i findNextSafePoint(size_t start_idx) const;
    size_t findClosestIndex(const std::vector<Eigen::Vector2i>& path,
                           const Eigen::Vector2i& target_pos) const;

    // 自适应密度调整函数
    double calculateEnvironmentComplexity(const Eigen::Vector2i& pos) const;
    double adaptiveStepAdjustment(const Eigen::Vector2i& pos, double base_step_m) const;

    // 辅助函数
    Eigen::Vector2i interpolateOnGrid(const Eigen::Vector2i& start,
                                     const Eigen::Vector2i& end, double t) const;
    // 删除路径中的回路：任何重复访问同一个格子的中间部分会被剪掉
    std::vector<Eigen::Vector2i> removePathLoops(const std::vector<Eigen::Vector2i>& path) const;

    // 判断一个栅格是否不可通行（致命障碍）
    bool isCellOccupied(const Eigen::Vector2i& g) const;
    // 判断两栅格之间直线是否无碰（Bresenham 采样）
    bool isLineFree(const Eigen::Vector2i& a, const Eigen::Vector2i& b) const;
    // 几何简化：删除"多余的折线"，只保留关键转折点
    std::vector<Eigen::Vector2i> simplifyPath(const std::vector<Eigen::Vector2i>& path) const;

    // 小工具
    inline float clampf(float v, float lo, float hi) const {
    return std::max(lo, std::min(hi, v));
    }
    // 上一次成功规划时所使用的地图版本（用于重规划触发控制）
    int last_planned_map_version_;

    // --- 置信度时间滤波相关成员 ---
    std::vector<float> last_confidence_;  // 上一帧的置信度，用于时间平滑
    bool confidence_history_initialized_; // 是否已初始化置信度历史
    const float confidence_alpha_ = 0.7f; // 时间滤波系数：0.7*old + 0.3*new

    // 辅助函数：添加三角形到标记中
    void addTriangleToMarker(visualization_msgs::Marker& marker,
                           const geometry_msgs::Point& p1,
                           const geometry_msgs::Point& p2,
                           const geometry_msgs::Point& p3,
                           const std_msgs::ColorRGBA& color);

private:

    // 在当前地图 & 距障碍距离下，找到旧路径第一个“不安全”的点
    size_t findFirstUnsafeIndex(const std::vector<Eigen::Vector2i>& path_idx,
                                double safety_margin_m) const;

    // 构建到旧路径的距离场（米），供 A* 使用 soft cost
    void buildDistanceToOldPath(const std::vector<Eigen::Vector2i>& ref_path_idx);

    // 判断当前 last_published_path_ 的走廊是否仍然安全
    bool isCorridorStillSafe(double safety_margin_m) const;

    // 找到当前机器人在 path 上的最近点索引
    int closestIndexOnPath(const nav_msgs::Path& path,
                        const geometry_msgs::Point& p_world) const;

    void gridInfoCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void droneOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void planningLoop(const ros::TimerEvent& event);
    void runPlanningCycle();
    void publishPathVisualization(const std::vector<Eigen::Vector2i>& path_indices, const std::vector<float>& half_width);
    void publishConfidenceVisualization(const std::vector<Eigen::Vector2i>& path_indices,
    const std::vector<float>& confidence,
    const std::vector<float>& half_width);
    void publishAstarRawMarker(const std::vector<Eigen::Vector2i>& path_indices);
    void publishGrpCenterlineMarker(const std::vector<geometry_msgs::Pose>& poses);
    std::vector<Eigen::Vector2i> pathMsgToIndices(const nav_msgs::Path& path) const;

    Eigen::Vector2i findExplorationGoal();
    void inflateCostmap();
    bool isPathStillValid();
    double distanceToPath();
    std::vector<Eigen::Vector2i> findPath(const Eigen::Vector2i& start, const Eigen::Vector2i& goal,
                                         const std::vector<bool>* blocked_cells = nullptr);
    double heuristic(const Eigen::Vector2i& a, const Eigen::Vector2i& b);
    double getCost(const Eigen::Vector2i& pos)const;
    double getCostAtOffset(int gx, int gy, double theta, double offset) const;
    double getCostAtOffsetPrecise(int gx, int gy, double theta, double offset) const;
    double getActualCorridorWidth(const geometry_msgs::Point& position) const;
    bool isObstacle(const Eigen::Vector2i& pos) const;

    // Corridor obstacle detection and dynamic adjustment functions
    bool doesCorridorIntersectObstacle(const geometry_msgs::Point& position, double half_width, int num_samples = 16) const;
    double getSafeCorridorWidth(const geometry_msgs::Point& position, double initial_half_width,
                               double min_half_width = 0.2, double max_half_width = 5.0) const;
    double calculateOptimalCorridorWidth(const geometry_msgs::Point& position) const;
    Eigen::Vector2i worldToGrid(const geometry_msgs::Point& point)const;
    geometry_msgs::Point gridToWorld(const Eigen::Vector2i& grid_pos) const;
    void publishPath(const std::vector<Eigen::Vector2i>& path_indices, const Eigen::Vector2i& goal_idx);
    void publishPathMarker(const nav_msgs::Path& path);
    void publishInflatedCostmap();
    void publishGoalMarker(bool delete_marker = false);  // 目标点的可视化
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber sub_grid_info_;
    ros::Subscriber sub_goal_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_drone_odom_;
    ros::Publisher pub_path_;
    ros::Publisher pub_path_marker_;
    ros::Publisher pub_goal_marker_;
    ros::Publisher pub_corridor_marker_; // 走廊可视化标记
    ros::Publisher pub_inflated_costmap_; // 膨胀代价地图发布器
    ros::Timer planning_timer_;

    nav_msgs::Path last_published_path_;

    // 路径滚动更新：机器人沿路径行驶时，裁剪已走过的前缀，使 /plan 的起点始终贴近当前位姿
    bool rolling_replan_enable_ = true;
    int  rolling_replan_keep_points_behind_ = 0; // 允许保留离机器人最近点之前的若干点（防抖/平滑）

    // 短段拼接式重规划：每移动一段距离，就只重规划“当前位姿 -> 前方 lookahead 点”，并复用旧路径剩余部分
    // 目的：路径会随行驶更新，但不会每次完全抛弃旧路径，减少抖动/跳变。
    bool stitch_replan_enable_ = false;
    double stitch_replan_trigger_distance_m_ = 0.2;   // m，移动超过该距离触发一次
    double stitch_replan_lookahead_distance_m_ = 3.0; // m，旧路径前方 lookahead 距离（拼接点）
    geometry_msgs::Point last_stitch_replan_position_;
    bool stitch_replan_baseline_valid_ = false;

    geometry_msgs::PoseStamped current_goal_;
    bool goal_received_ = false;
    bool new_goal_trigger_ = false;
    bool is_planning_active_ = false;

    // 增量式有效目标点搜索所需的状态变量
    bool is_searching_for_valid_goal_ = false;
    ros::Time search_start_time_;
    Eigen::Vector2i search_center_;
    double search_radius_; // 当前搜索半径
    std::mt19937 random_generator_; // 随机数生成器

    geometry_msgs::Pose current_robot_pose_;
    bool pose_received_ = false;

    geometry_msgs::Pose current_drone_pose_;
    bool drone_pose_received_ = false;

    // 地图核心数据
    nav_msgs::OccupancyGrid::ConstPtr grid_info_;
    std::vector<double> cost_map_;
    std::vector<float> distance_to_obstacle_; // 新增：存储到最近障碍物的距离场
    bool map_received_ = false;
    // 到"旧路径"的距离场（米），用于 soft cost
    std::vector<float> distance_to_old_path_;

    // 稳定栅格相关成员变量，用于防止地图闪烁
    std::vector<StableCell> stable_cells_;
    int occ_confirm_frames_;   // 连续观测为占据的确认帧数 (N_occ)
    int free_confirm_frames_;  // 连续观测为空闲的确认帧数 (N_free)
    int occupied_threshold_;   // OccupancyGrid >= 该值认为“占据/障碍”
    // 是否启用旧路径贴近偏置 & 权重（调这个影响新路径粘旧路的程度）
    bool   ref_path_bias_enable_;
    double ref_path_bias_weight_;
    // 参数
    double inflation_radius_;
    double unknown_space_cost_;
    double path_marker_width_;  // 路径标记的线宽度
    double planning_frequency_;
    double replanning_distance_threshold_;
    double obstacle_distance_cost_weight_; // 离障碍物距离成本的权重
    double turn_penalty_cost_weight_;     // 拐弯惩罚成本的权重
    double goal_weight_;                  // 最终目标在探索得分中的权重
    double drone_pos_weight_;             // 无人机位置在探索得分中的权重
    std::string odom_topic_;
    std::string robot_frame_;
    std::string world_frame_;
};

} // namespace fitplane_planner

#endif // PATH_PLANNER_H 
