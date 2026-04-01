#include "fitplane_planner/path_planner.h"
#include <queue> // for std::queue, used in distance field calculation
#include <algorithm>
#include <chrono>
#include <cmath>

namespace fitplane_planner 
{

namespace {
inline double msSince(std::chrono::steady_clock::time_point start) {
    using namespace std::chrono;
    return duration_cast<duration<double, std::milli>>(steady_clock::now() - start).count();
}
} // namespace

void PathPlanner::gridInfoCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    const auto t_cb = std::chrono::steady_clock::now();
    bool meta_changed = (!grid_info_) ||
                        (grid_info_->info.width  != msg->info.width) ||
                        (grid_info_->info.height != msg->info.height) ||
                        (grid_info_->info.resolution != msg->info.resolution) ||
                        (grid_info_->info.origin.position.x != msg->info.origin.position.x) ||
                        (grid_info_->info.origin.position.y != msg->info.origin.position.y);
    grid_info_ = msg;
    const size_t N = static_cast<size_t>(msg->info.width) * msg->info.height;

    // --- 1) 初始化稳定栅格数组 ---
    if (stable_cells_.size() != N) {
        stable_cells_.assign(N, StableCell{ -1, 0, 0 });  // 初始全 unknown
        meta_changed = true;
    }

    // --- 2) 先用稳定占据构建 cost_map_ 的基础框架 ---
    cost_map_.assign(N, 0.0);

    const auto t_stable = std::chrono::steady_clock::now();
    bool any_state_changed = false;
    for (size_t i = 0; i < N; ++i) {
        int meas_occ = static_cast<int>(msg->data[i]);
        if (meas_occ < -1) meas_occ = -1;
        if (meas_occ > 100) meas_occ = 100;
        StableCell &cell = stable_cells_[i];
        const int8_t prev_state = cell.state;

        // 根据传感器观测更新计数器 & 状态
        if (meas_occ >= occupied_threshold_) {
            // 连续占据计数
            if (cell.occ_cnt < 255) cell.occ_cnt++;
            cell.free_cnt = 0;
            if (cell.occ_cnt >= occ_confirm_frames_) {
                cell.state = 100;
            }
        } else if (meas_occ >= 0) {
            // 连续空闲计数
            if (cell.free_cnt < 255) cell.free_cnt++;
            cell.occ_cnt = 0;
            if (cell.free_cnt >= free_confirm_frames_) {
                cell.state = 0;
            }
        } else {
            // meas_occ == -1（未知）：
            //   这里选择"不动计数、不改状态"，让状态更多由 100/0 决定
        }

        if (cell.state != prev_state) {
            any_state_changed = true;
        }

        // --- 3) 用稳定 state → cost_map_ ---
        if (cell.state == 100) {
            // 规划眼里是致命障碍
            cost_map_[i] = -1.0;
        } else if (cell.state == -1) {
            // 规划眼里未知区域：用 unknown_space_cost_ 表示
            cost_map_[i] = unknown_space_cost_;
        } else {
            // 0: free
            cost_map_[i] = 0.0;
        }
    }
    const double ms_stable = msSince(t_stable);

    // 语义上“地图没变”：稳定占据状态无变化且元信息未变。
    // 这可以避免 MapManager 周期发布同一张地图导致 map_version_ 每帧增长、从而禁用路径复用。
    if (map_received_ && !meta_changed && !any_state_changed) {
        ROS_DEBUG_THROTTLE(2.0, "Grid map unchanged (stable occupancy unchanged). Skip inflate+replan.");
        return;
    }

    // 膨胀代价地图
    const auto t_inflate = std::chrono::steady_clock::now();
    inflateCostmap();
    const double ms_inflate = msSince(t_inflate);

    // 发布膨胀后的代价地图用于可视化
    const auto t_pub_costmap = std::chrono::steady_clock::now();
    publishInflatedCostmap();
    const double ms_pub_costmap = msSince(t_pub_costmap);

    map_received_ = true;
    ROS_INFO_ONCE("Grid map received and costmap created/inflated.");
    last_map_update_time_ = ros::Time::now();
    map_version_++;  // ★ 地图语义变化时自增版本


    // 立即触发一次规划以获得即时响应
    const auto t_plan = std::chrono::steady_clock::now();
    runPlanningCycle();
    const double ms_plan = msSince(t_plan);

    ROS_INFO_THROTTLE(1.0,
                      "Timing(ms): gridInfoCallback total=%.1f stable=%.1f inflate=%.1f pub_costmap=%.1f plan=%.1f N=%zu",
                      msSince(t_cb), ms_stable, ms_inflate, ms_pub_costmap, ms_plan, N);
}




void PathPlanner::inflateCostmap() {
    if (!grid_info_) return;

    const auto t_total = std::chrono::steady_clock::now();
    std::vector<double> inflated_cost_map = cost_map_;
    int inflation_cells = static_cast<int>(inflation_radius_ / grid_info_->info.resolution);
    if (inflation_cells == 0) return; // 无需膨胀

    int width = grid_info_->info.width;
    int height = grid_info_->info.height;

    const auto t_inflate = std::chrono::steady_clock::now();
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            // 如果当前单元格是障碍物
            if (cost_map_[y * width + x] < 0) {
                // 在膨胀半径内迭代
                for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
                    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;

                        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                            double dist = std::sqrt(dx*dx + dy*dy);
                            if (dist <= inflation_cells) {
                                // 改进的非线性成本衰减 - 接近障碍物时成本上升更快
                                double inflation_cost = 100.0 * std::exp(-2.0 * dist / inflation_cells);
                                int n_idx = ny * width + nx;
                                if (inflated_cost_map[n_idx] >= 0) { // 不要覆盖致命障碍物
                                     inflated_cost_map[n_idx] = std::max(inflation_cost, inflated_cost_map[n_idx]);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    cost_map_ = inflated_cost_map;
    const double ms_inflate = msSince(t_inflate);

    // --- 新增：计算距离场 ---
    const auto t_df = std::chrono::steady_clock::now();
    distance_to_obstacle_.assign(cost_map_.size(), std::numeric_limits<float>::max());
    std::queue<Eigen::Vector2i> q;

    // 定义高代价阈值：超过这个值的膨胀栅格也当作障碍源
    const double high_cost_threshold = 80.0;  // 恢复原始阈值

    // 初始化队列，所有障碍物点和高代价膨胀点距离为0
    for(int y=0; y<height; ++y){
        for(int x=0; x<width; ++x){
            const int idx = y * width + x;
            const double cost = cost_map_[idx];

            // 致命障碍 OR 高代价膨胀区都作为障碍源
            if(cost < 0 || cost > high_cost_threshold){
                distance_to_obstacle_[idx] = 0;
                q.push(Eigen::Vector2i(x,y));
            }
        }
    }

    // 执行类BFS算法 (更准确地说是类Dijkstra的传播)
    int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};
    double d_dist[] = {1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414};

    while(!q.empty()){
        Eigen::Vector2i curr = q.front();
        q.pop();

        int curr_idx = curr.y() * width + curr.x();

        for(int i=0; i<8; ++i){
            Eigen::Vector2i next(curr.x() + dx[i], curr.y() + dy[i]);
            if(next.x() >= 0 && next.x() < width && next.y() >= 0 && next.y() < height){
                int next_idx = next.y() * width + next.x();
                if(distance_to_obstacle_[next_idx] > distance_to_obstacle_[curr_idx] + d_dist[i]){
                    distance_to_obstacle_[next_idx] = distance_to_obstacle_[curr_idx] + d_dist[i];
                    q.push(next);
                }
            }
        }
    }

    const double ms_df = msSince(t_df);
    ROS_INFO_THROTTLE(1.0,
                      "Timing(ms): inflateCostmap total=%.1f inflate=%.1f dist_field=%.1f (W=%d H=%d cells=%zu)",
                      msSince(t_total), ms_inflate, ms_df, width, height, cost_map_.size());
}

double PathPlanner::getCost(const Eigen::Vector2i& pos)const {
    if (pos.x() < 0 || pos.x() >= grid_info_->info.width || pos.y() < 0 || pos.y() >= grid_info_->info.height) {
        return -1.0; // 越界
    }
    int index = pos.y() * grid_info_->info.width + pos.x();
    return cost_map_[index];
}



double PathPlanner::getCostAtOffset(int gx, int gy, double theta, double offset) const {
    // 计算垂直于路径方向的偏移点代价
    const double res = grid_info_->info.resolution;
    const double ox  = grid_info_->info.origin.position.x;
    const double oy  = grid_info_->info.origin.position.y;

    // gx/gy 表示栅格索引，这里使用栅格中心点作为基准
    const double world_x = (static_cast<double>(gx) + 0.5) * res + ox;
    const double world_y = (static_cast<double>(gy) + 0.5) * res + oy;

    const double offset_world_x = world_x + offset * (-std::sin(theta));
    const double offset_world_y = world_y + offset * ( std::cos(theta));

    const int offset_gx = static_cast<int>(std::floor((offset_world_x - ox) / res));
    const int offset_gy = static_cast<int>(std::floor((offset_world_y - oy) / res));

    if (!grid_info_ || offset_gx < 0 || offset_gx >= grid_info_->info.width ||
        offset_gy < 0 || offset_gy >= grid_info_->info.height) {
        return -1.0; // 超出地图边界视为障碍
    }

    int index = offset_gy * grid_info_->info.width + offset_gx;
    if (index < 0 || index >= static_cast<int>(cost_map_.size())) {
        return -1.0;
    }

    return cost_map_[index];
}



double PathPlanner::getCostAtOffsetPrecise(int gx, int gy, double theta, double offset) const {
    // 高精度边界检查函数，使用双线性插值和多点采样

    // 计算世界坐标并进行精确转换
    const double res = grid_info_->info.resolution;
    const double ox  = grid_info_->info.origin.position.x;
    const double oy  = grid_info_->info.origin.position.y;

    // gx/gy 表示栅格索引，这里使用栅格中心点作为基准
    const double world_x = (static_cast<double>(gx) + 0.5) * res + ox;
    const double world_y = (static_cast<double>(gy) + 0.5) * res + oy;

    const double offset_world_x = world_x + offset * (-std::sin(theta));
    const double offset_world_y = world_y + offset * ( std::cos(theta));

    // 精确转换回栅格坐标
    const double offset_gx_f = (offset_world_x - ox) / res;
    const double offset_gy_f = (offset_world_y - oy) / res;

    // 双线性插值处理
    int x0 = std::floor(offset_gx_f);
    int y0 = std::floor(offset_gy_f);
    int x1 = x0 + 1;
    int y1 = y0 + 1;

    // 边界检查
    if (x0 < 0 || x1 >= grid_info_->info.width || y0 < 0 || y1 >= grid_info_->info.height) {
        return -1.0;
    }

    // 获取4个邻近点代价
    double c00 = cost_map_[y0 * grid_info_->info.width + x0];
    double c10 = cost_map_[y0 * grid_info_->info.width + x1];
    double c01 = cost_map_[y1 * grid_info_->info.width + x0];
    double c11 = cost_map_[y1 * grid_info_->info.width + x1];

    // 如果任何邻近点是致命障碍物，返回致命
    if (c00 < 0 || c10 < 0 || c01 < 0 || c11 < 0) {
        return -1.0;
    }

    // 双线性插值
    double wx = offset_gx_f - x0;
    double wy = offset_gy_f - y0;
    double c0 = c00 * (1 - wx) + c10 * wx;
    double c1 = c01 * (1 - wx) + c11 * wx;
    return c0 * (1 - wy) + c1 * wy;
}



bool PathPlanner::isObstacle(const Eigen::Vector2i& pos) const {
    // 统一的障碍物检测标准：cost < 0
    if (!grid_info_) return true;

    if (pos.x() < 0 || pos.x() >= grid_info_->info.width ||
        pos.y() < 0 || pos.y() >= grid_info_->info.height) {
        return true; // 越界视为障碍
    }

    return getCost(pos) < 0.0;
}

Eigen::Vector2i PathPlanner::worldToGrid(const geometry_msgs::Point& point)const {
    Eigen::Vector2i grid_pos;
    // 注意：C++ int 转换对负数是“向 0 截断”，会导致负坐标/边界附近索引错位
    grid_pos.x() = static_cast<int>(std::floor((point.x - grid_info_->info.origin.position.x) / grid_info_->info.resolution));
    grid_pos.y() = static_cast<int>(std::floor((point.y - grid_info_->info.origin.position.y) / grid_info_->info.resolution));
    return grid_pos;
}



geometry_msgs::Point PathPlanner::gridToWorld(const Eigen::Vector2i& grid_pos) const {
    geometry_msgs::Point world_pos;
    world_pos.x = grid_pos.x() * grid_info_->info.resolution + grid_info_->info.origin.position.x + grid_info_->info.resolution/2.0;
    world_pos.y = grid_pos.y() * grid_info_->info.resolution + grid_info_->info.origin.position.y + grid_info_->info.resolution/2.0;
    
    // 忽略地图中的高度信息，将路径点的Z坐标设为0
    world_pos.z = 0.0;
    return world_pos;
}

void PathPlanner::publishInflatedCostmap() {
    if (!grid_info_ || cost_map_.empty()) {
        return;
    }

    nav_msgs::OccupancyGrid grid_msg;
    grid_msg.header.frame_id = world_frame_;
    grid_msg.header.stamp = ros::Time::now();
    grid_msg.info.resolution = grid_info_->info.resolution;
    grid_msg.info.width = grid_info_->info.width;
    grid_msg.info.height = grid_info_->info.height;
    grid_msg.info.origin.position.x = grid_info_->info.origin.position.x;
    grid_msg.info.origin.position.y = grid_info_->info.origin.position.y;
    grid_msg.info.origin.position.z = 0.0;
    grid_msg.info.origin.orientation.w = 1.0;

    grid_msg.data.resize(cost_map_.size());
    for (size_t i = 0; i < cost_map_.size(); ++i) {
        if (cost_map_[i] < 0) {
            grid_msg.data[i] = 100;  // 致命障碍物（负值表示原始障碍物）
        } else if (cost_map_[i] == unknown_space_cost_) {
            grid_msg.data[i] = -1;   // 未知区域
        } else {
            // cost_map_ 已经是膨胀后的值，范围在0-100
            // 直接映射到OccupancyGrid的0-100范围
            int8_t cost = static_cast<int8_t>(std::min(100.0, std::max(0.0, cost_map_[i])));
            grid_msg.data[i] = cost;
        }
    }

    pub_inflated_costmap_.publish(grid_msg);
}

} // namespace fitplane_planner
