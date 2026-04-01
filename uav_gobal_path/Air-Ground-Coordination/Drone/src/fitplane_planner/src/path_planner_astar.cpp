#include "fitplane_planner/path_planner.h"
#include <algorithm>
#include <chrono>

namespace fitplane_planner 
{

namespace {
inline double msSince(std::chrono::steady_clock::time_point start) {
    using namespace std::chrono;
    return duration_cast<duration<double, std::milli>>(steady_clock::now() - start).count();
}
} // namespace

// std::vector<Eigen::Vector2i> PathPlanner::findPath(const Eigen::Vector2i& start, const Eigen::Vector2i& goal) {
//     std::priority_queue<Node*, std::vector<Node*>, Node::Compare> open_set;
//     std::vector<Node*> all_nodes;
    
//     std::vector<bool> closed_set(grid_info_->info.width * grid_info_->info.height, false);

//     Node* start_node = new Node(start, 0, heuristic(start, goal), nullptr);
//     open_set.push(start_node);
//     all_nodes.push_back(start_node);

//     while (!open_set.empty()) {
//         Node* current = open_set.top();
//         open_set.pop();

//         if (current->position == goal) {
//             std::vector<Eigen::Vector2i> path;
//             Node* temp = current;
//             while (temp != nullptr) {
//                 path.push_back(temp->position);
//                 temp = temp->parent;
//             }
//             std::reverse(path.begin(), path.end());
            
//             // Clean up memory
//             for(Node* n : all_nodes) delete n;

//             return path;
//         }

//         int current_idx = current->position.y() * grid_info_->info.width + current->position.x();
//         if(closed_set[current_idx]) {
//             // This node is already processed, but was pushed to open_set again
//             // with a higher cost. Just ignore it.
//             continue;
//         }
//         closed_set[current_idx] = true;

//         // 探索邻居 (8个方向)
//         for (int dx = -1; dx <= 1; ++dx) {
//             for (int dy = -1; dy <= 1; ++dy) {
//                 if (dx == 0 && dy == 0) continue;

//                 Eigen::Vector2i neighbor_pos(current->position.x() + dx, current->position.y() + dy);

//                 // 检查边界
//                 if (neighbor_pos.x() < 0 || neighbor_pos.x() >= grid_info_->info.width ||
//                     neighbor_pos.y() < 0 || neighbor_pos.y() >= grid_info_->info.height) {
//                     continue;
//                 }

//                 int neighbor_idx = neighbor_pos.y() * grid_info_->info.width + neighbor_pos.x();

//                 // 检查是否在closed set中
//                 if (closed_set[neighbor_idx]) {
//                     continue;
//                 }
                
//                 // 检查是否是障碍物
//                 double neighbor_cost = getCost(neighbor_pos);
//                 if (neighbor_cost < 0) {
//                     continue;
//                 }

//                 double move_cost = (dx == 0 || dy == 0) ? 1.0 : 1.414; // 直线或斜线移动代价
                
//                 // --- 核心修改：新的代价计算 ---
//                 double traversability_cost = neighbor_cost;
                
//                 // 离障碍物距离代价 (距离越近，代价越高)
//                 float dist_to_obs = distance_to_obstacle_[neighbor_idx];
//                 double obstacle_dist_cost = 0.0;
//                 if (dist_to_obs < inflation_radius_ / grid_info_->info.resolution) {
//                     obstacle_dist_cost = 1.0 - (dist_to_obs / (inflation_radius_ / grid_info_->info.resolution));
//                 }

//                 // 拐弯惩罚代价
//                 double turn_penalty = 0.0;
//                 if(current->parent != nullptr){
//                     int prev_dx = current->position.x() - current->parent->position.x();
//                     int prev_dy = current->position.y() - current->parent->position.y();
//                     if(dx != prev_dx || dy != prev_dy){
//                         turn_penalty = 1.0;
//                     }
//                 }
// 				double ref_path_cost = 0.0;
// 				if (ref_path_bias_enable_ &&
// 						distance_to_old_path_.size() == cost_map_.size()) {

// 						float d_ref = distance_to_old_path_[neighbor_idx];  // 米
// 						if (std::isfinite(d_ref)) {
// 								// 你可以稍微做个压缩，比如用 1 - exp(-d)，这里先用线性
// 								ref_path_cost = ref_path_bias_weight_ * static_cast<double>(d_ref);
// 						}
// 				}

// 				// ---------- 总代价 ----------
// 				// 原有几项还是乘在 move_cost 上，ref_path_cost 直接加（你也可以乘在 move_cost 上） 
// 				double combined_cost = move_cost * (1.0
// 																						+ traversability_cost
// 																						+ obstacle_distance_cost_weight_ * obstacle_dist_cost
// 																						+ turn_penalty_cost_weight_ * turn_penalty)
// 															+ ref_path_cost;

// 				double g_cost = current->g_cost + combined_cost;
//                 // double combined_cost = move_cost * (1.0 + traversability_cost + 
//                 //                                    obstacle_distance_cost_weight_ * obstacle_dist_cost +
//                 //                                    turn_penalty_cost_weight_ * turn_penalty);

//                 // double g_cost = current->g_cost + combined_cost;
//                 double h_cost = heuristic(neighbor_pos, goal);

//                 Node* neighbor_node = new Node(neighbor_pos, g_cost, h_cost, current);
//                 open_set.push(neighbor_node);
//                 all_nodes.push_back(neighbor_node);
//             }
//         }
//     }
    
//     // Clean up memory if no path is found
//     for(Node* n : all_nodes) delete n;

//     return {}; // 返回空路径
// }
std::vector<Eigen::Vector2i> PathPlanner::findPath(const Eigen::Vector2i& start,
                                                   const Eigen::Vector2i& goal,
                                                   const std::vector<bool>* blocked_cells)
{
    const auto t_astar = std::chrono::steady_clock::now();
    std::vector<Eigen::Vector2i> empty_result;
    if (!grid_info_) return empty_result;

    const int W = grid_info_->info.width;
    const int H = grid_info_->info.height;
    if (start.x() < 0 || start.x() >= W || start.y() < 0 || start.y() >= H ||
        goal.x()  < 0 || goal.x()  >= W || goal.y()  < 0 || goal.y()  >= H) {
        return empty_result;
    }

    // --- 局部窗口（bounding box） ---
    // 以 start 和 goal 为中心，向外扩 margin_m 米，只在窗口中搜索
    const double margin_m    = 10.0; // 可做成 rosparam
    const int    margin_cell = std::max(5, static_cast<int>(margin_m / grid_info_->info.resolution));

    int min_x = std::max(0, std::min(start.x(), goal.x()) - margin_cell);
    int max_x = std::min(W - 1, std::max(start.x(), goal.x()) + margin_cell);
    int min_y = std::max(0, std::min(start.y(), goal.y()) - margin_cell);
    int max_y = std::min(H - 1, std::max(start.y(), goal.y()) + margin_cell);

    auto inWindow = [&](int x, int y) {
        return (x >= min_x && x <= max_x && y >= min_y && y <= max_y);
    };

    if (!inWindow(start.x(), start.y()) || !inWindow(goal.x(), goal.y())) {
        // 理论上不会出现，但保险起见
        min_x = 0; max_x = W - 1;
        min_y = 0; max_y = H - 1;
    }

    const int start_idx = start.y() * W + start.x();
    const int goal_idx  = goal.y()  * W + goal.x();

    std::vector<GridNode> nodes(W * H);

    NodeCompare cmp(&nodes);
    std::priority_queue<int, std::vector<int>, NodeCompare> open_set(cmp);

    // 初始化起点
    nodes[start_idx].g       = 0.0;
    nodes[start_idx].h       = heuristic(start, goal);
    nodes[start_idx].parent  = -1;
    nodes[start_idx].in_open = true;
    nodes[start_idx].closed  = false;
    open_set.push(start_idx);

    const int   dirs[8][2] = {
        {-1, 0},{1, 0},{0,-1},{0, 1},
        {-1,-1},{-1, 1},{1,-1},{1, 1}
    };
    const double move_step[8] = {1.0,1.0,1.0,1.0,1.414,1.414,1.414,1.414};

    int expanded = 0;
    int pushed = 1; // start pushed once
    int relax = 0;

    while (!open_set.empty()) {
        int cur_idx = open_set.top();
        open_set.pop();

        GridNode& cur_node = nodes[cur_idx];
        if (cur_node.closed) continue;
        cur_node.closed = true;
        ++expanded;

        if (cur_idx == goal_idx) {
            // 回溯路径
            std::vector<Eigen::Vector2i> path_rev;
            int idx = goal_idx;
            while (idx != -1) {
                int x = idx % W;
                int y = idx / W;
                path_rev.emplace_back(x, y);
                idx = nodes[idx].parent;
            }
            std::reverse(path_rev.begin(), path_rev.end());
            ROS_INFO_THROTTLE(1.0,
                              "Timing(ms): astar=%.1f expanded=%d pushed=%d relax=%d window=[%d..%d,%d..%d] len=%zu",
                              msSince(t_astar), expanded, pushed, relax, min_x, max_x, min_y, max_y, path_rev.size());
            return path_rev;
        }

        int cx = cur_idx % W;
        int cy = cur_idx / W;

        for (int k = 0; k < 8; ++k) {
            int nx = cx + dirs[k][0];
            int ny = cy + dirs[k][1];

            if (!inWindow(nx, ny)) continue;  // 限制在局部窗口内
            if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;

            int nb_idx = ny * W + nx;

            // === 新增：利用 blocked_cells 掩码禁止访问某些栅格 ===
            if (blocked_cells && nb_idx >= 0 &&
                nb_idx < static_cast<int>(blocked_cells->size()) &&
                (*blocked_cells)[nb_idx]) {
                continue;   // 旧前缀中的格子（除了拼接点）会被挡掉
            }

            GridNode& nb_node = nodes[nb_idx];

            if (nb_node.closed) continue;

            Eigen::Vector2i nb_pos(nx, ny);
            // 禁止对角切角(corner-cutting)：对角步时，两侧正交相邻格只要有一个是障碍，就不允许该步
            if (dirs[k][0] != 0 && dirs[k][1] != 0) {
                if (getCost(Eigen::Vector2i(cx, ny)) < 0.0 ||
                    getCost(Eigen::Vector2i(nx, cy)) < 0.0) {
                    continue;
                }
            }
            double neighbor_cost = getCost(nb_pos);
            if (neighbor_cost < 0.0) continue; // 致命障碍物

            // 基本移动代价
            double move_cost = move_step[k];

            // traversability cost
            double traversability_cost = neighbor_cost;

            // 障碍距离代价
            double obstacle_dist_cost = 0.0;
            if (!distance_to_obstacle_.empty()) {
                float dcell = distance_to_obstacle_[nb_idx];
                if (std::isfinite(dcell)) {
                    double max_dcell = inflation_radius_ / grid_info_->info.resolution;
                    if (dcell < max_dcell) {
                        obstacle_dist_cost = 1.0 - (dcell / max_dcell);
                    }
                }
            }

            // 拐弯惩罚
            double turn_penalty = 0.0;
            if (cur_node.parent != -1) {
                int px = cur_node.parent % W;
                int py = cur_node.parent / W;
                int prev_dx = cx - px;
                int prev_dy = cy - py;
                if (prev_dx != dirs[k][0] || prev_dy != dirs[k][1]) {
                    turn_penalty = 1.0;
                }
            }

            // 贴近旧路径的 soft cost
            double ref_path_cost = 0.0;
            if (ref_path_bias_enable_ &&
                !distance_to_old_path_.empty() &&
                distance_to_old_path_.size() == nodes.size()) {

                float d_ref = distance_to_old_path_[nb_idx]; // 米
                if (std::isfinite(d_ref)) {
                    ref_path_cost = ref_path_bias_weight_ * static_cast<double>(d_ref);
                }
            }

            // 总代价
            double combined_cost = move_cost * (1.0
                                                + traversability_cost
                                                + obstacle_distance_cost_weight_ * obstacle_dist_cost
                                                + turn_penalty_cost_weight_       * turn_penalty)
                                   + ref_path_cost;

            double new_g = cur_node.g + combined_cost;

            if (new_g < nb_node.g) {
                nb_node.g      = new_g;
                nb_node.h      = heuristic(nb_pos, goal);
                nb_node.parent = cur_idx;
                ++relax;

                if (!nb_node.in_open) {
                    nb_node.in_open = true;
                    open_set.push(nb_idx);
                    ++pushed;
                } else {
                    // 在没有 decrease-key 的情况下，再 push 一次即可，旧条目会因 closed/g 值被忽略
                    open_set.push(nb_idx);
                    ++pushed;
                }
            }
        }
    }

    // 搜索失败
    ROS_INFO_THROTTLE(1.0,
                      "Timing(ms): astar=%.1f expanded=%d pushed=%d relax=%d window=[%d..%d,%d..%d] (no path)",
                      msSince(t_astar), expanded, pushed, relax, min_x, max_x, min_y, max_y);
    return empty_result;
}

// double PathPlanner::heuristic(const Eigen::Vector2i& a, const Eigen::Vector2i& b) {
//     // 使用曼哈顿距离作为启发函数，对于网格地图更优
//     return std::abs(a.x() - b.x()) + std::abs(a.y() - b.y());
// }
double PathPlanner::heuristic(const Eigen::Vector2i& a, const Eigen::Vector2i& b) {
    // 八邻域更适配 octile
    double dx = std::abs(a.x() - b.x());
    double dy = std::abs(a.y() - b.y());
    const double D  = 1.0;
    const double D2 = 1.41421356237;
    return D * (dx + dy) + (D2 - 2.0 * D) * std::min(dx, dy);
}

bool PathPlanner::isCellOccupied(const Eigen::Vector2i& g) const
{
    if (!grid_info_) return true; // 没地图就保守认为占据

    if (g.x() < 0 || g.x() >= grid_info_->info.width ||
        g.y() < 0 || g.y() >= grid_info_->info.height) {
        return true; // 越界当成障碍
    }

    double c = getCost(g);
    // 注意：你的 getCost 对致命障碍返回 < 0
    return (c < 0.0);
}



bool PathPlanner::isLineFree(const Eigen::Vector2i& a,
                             const Eigen::Vector2i& b) const
{
    // Bresenham + supercover：对角步同时检查两侧格子，避免“切角穿障”漏检
    int x0 = a.x();
    int y0 = a.y();
    int x1 = b.x();
    int y1 = b.y();

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    // 起点也需要检查
    if (isCellOccupied(Eigen::Vector2i(x0, y0))) {
        return false;
    }

    while (!(x0 == x1 && y0 == y1)) {
        const int prev_x = x0;
        const int prev_y = y0;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }

        if (x0 != prev_x && y0 != prev_y) {
            if (isCellOccupied(Eigen::Vector2i(x0, prev_y))) return false;
            if (isCellOccupied(Eigen::Vector2i(prev_x, y0))) return false;
        }

        if (isCellOccupied(Eigen::Vector2i(x0, y0))) {
            return false;
        }
    }

    return true;
}



std::vector<Eigen::Vector2i> PathPlanner::simplifyPath(
    const std::vector<Eigen::Vector2i>& path) const
{
    if (path.size() <= 2) {
        return path;  // 0,1,2个点没啥可简化的
    }

    std::vector<Eigen::Vector2i> out;
    out.reserve(path.size());

    // anchor = 上一个"保留的点"在原路径中的下标
    size_t anchor = 0;
    out.push_back(path[anchor]);

    // j 从 anchor+1 开始往前试探：
    // 只要 anchor -> path[j] 的直线无碰撞，就继续往前看；
    // 一旦失败，就把 j-1 作为新的关键点加入 out，anchor 移到 j-1。
    while (true) {
        size_t next = anchor + 1;
        size_t farthest = next;

        // 尝试尽可能远的点
        for (size_t j = next + 1; j < path.size(); ++j) {
            if (isLineFree(path[anchor], path[j])) {
                farthest = j;
            } else {
                break;  // 再往后肯定更差，直接停
            }
        }

        // 如果一个格子都走不远，说明被障碍卡住，只能接受 next
        if (farthest == anchor + 1) {
            out.push_back(path[next]);
            anchor = next;
        } else {
            out.push_back(path[farthest]);
            anchor = farthest;
        }

        if (anchor >= path.size() - 1) {
            break;  // 已经到终点
        }
    }

    // 保证终点是最后一个
    if (out.back().x() != path.back().x() ||
        out.back().y() != path.back().y()) {
        out.push_back(path.back());
    }

    return out;
}

} // namespace fitplane_planner
