#include "fitplane_planner/path_planner.h"
#include <queue> // for std::queue, used in distance field calculation
#include <unordered_map> // for std::unordered_map, used in removePathLoops
#include <algorithm>

namespace fitplane_planner 
{

// 构建到旧路径的距离场（米）
// ref_path_idx：参考路径上的所有栅格索引（通常来自 last_published_path_）
void PathPlanner::buildDistanceToOldPath(const std::vector<Eigen::Vector2i>& ref_path_idx)
{
    if (!grid_info_ || ref_path_idx.empty() || !ref_path_bias_enable_) {
        distance_to_old_path_.clear();
        return;
    }

    const int    W   = grid_info_->info.width;
    const int    H   = grid_info_->info.height;
    const double res = grid_info_->info.resolution;

    distance_to_old_path_.assign(W * H, std::numeric_limits<float>::infinity());
    std::queue<Eigen::Vector2i> q;

    // 初始化：旧路径所有栅格点距离为 0，作为 BFS 源点
    for (const auto& p : ref_path_idx) {
        if (p.x() < 0 || p.x() >= W || p.y() < 0 || p.y() >= H) continue;
        int idx = p.y() * W + p.x();
        if (distance_to_old_path_[idx] > 0.0f) {
            distance_to_old_path_[idx] = 0.0f;
            q.push(p);
        }
    }

    if (q.empty()) {
        // 可能全越界了
        return;
    }

    // 八邻域 BFS（类似 distance_to_obstacle_ 的写法）
    const int    dx[8]    = {-1, 1, 0, 0, -1, -1,  1,  1};
    const int    dy[8]    = { 0, 0,-1, 1, -1,  1, -1,  1};
    const double d_step[8]= { 1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414};

    while (!q.empty()) {
        Eigen::Vector2i cur = q.front();
        q.pop();
        int cur_idx = cur.y() * W + cur.x();
        float cur_dist = distance_to_old_path_[cur_idx];

        for (int k = 0; k < 8; ++k) {
            int nx = cur.x() + dx[k];
            int ny = cur.y() + dy[k];
            if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;

            int nb_idx = ny * W + nx;
            float new_dist = cur_dist + static_cast<float>(d_step[k] * res);

            if (new_dist < distance_to_old_path_[nb_idx]) {
                distance_to_old_path_[nb_idx] = new_dist;
                q.push(Eigen::Vector2i(nx, ny));
            }
        }
    }

    ROS_DEBUG_THROTTLE(2.0, "Distance field to old path built (bias weight=%.3f).",
                       ref_path_bias_weight_);
}



// >>> 新增：在当前地图 & 距障碍距离下，找到旧路径第一个"不安全"的点
// 判定条件：
//   1) 路径中心落在致命障碍物上（getCost < 0）
//   2) 到障碍物的 clearance < safety_margin_m
// 返回值：第一个不安全点的索引；如果全都安全，返回 path_idx.size()
size_t PathPlanner::findFirstUnsafeIndex(
    const std::vector<Eigen::Vector2i>& path_idx,
    double safety_margin_m) const
{
    if (!grid_info_ || path_idx.empty()) return 0;

    const int    W   = grid_info_->info.width;
    const int    H   = grid_info_->info.height;
    const double res = grid_info_->info.resolution;

    for (size_t i = 0; i < path_idx.size(); ++i) {
        const auto& p = path_idx[i];

        if (p.x() < 0 || p.x() >= W || p.y() < 0 || p.y() >= H) {
            ROS_WARN("[FIND_UNSAFE] Path point %zu out of bounds: (%d,%d), map size=%dx%d",
                     i, p.x(), p.y(), W, H);
            return i;   // 越界视为不安全
        }

        // 1) 路径中心是否进入致命障碍
        double cost = getCost(p);
        if (cost < 0) {
            ROS_WARN("[FIND_UNSAFE] Path point %zu at (%d,%d) on obstacle (cost=%.2f)",
                     i, p.x(), p.y(), cost);
            return i;
        }

        // 2) 走廊 / clearence 是否过窄
        const int idx = p.y() * W + p.x();
        float dcell   = distance_to_obstacle_[idx];

        if (!std::isfinite(dcell)) {
            // 没有致命障碍，认为足够安全
            continue;
        }

        double clearance_m = dcell * res;
        if (clearance_m < safety_margin_m) {
            ROS_WARN("[FIND_UNSAFE] Path point %zu at (%d,%d) clearance too small: %.2fm < %.2fm",
                     i, p.x(), p.y(), clearance_m, safety_margin_m);
            return i;
        }
    }

    ROS_INFO("[FIND_UNSAFE] All %zu path points are safe (safety_margin=%.2fm)",
             path_idx.size(), safety_margin_m);
    return path_idx.size();  // 全部安全
}

// <<< 新增结束

float PathPlanner::computeUnknownRatio(const Eigen::Vector2i& c, int r) const {
    if (!grid_info_ || r <= 0 || stable_cells_.empty()) return 0.0f;

    const int W = grid_info_->info.width;
    const int H = grid_info_->info.height;

    int total = 0;
    int unknown = 0;

    for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
            if (dx * dx + dy * dy > r * r) continue;

            const int x = c.x() + dx;
            const int y = c.y() + dy;
            if (x < 0 || x >= W || y < 0 || y >= H) continue;

            ++total;
            const int idx = y * W + x;

            // ★ 关键：直接使用 stable_cells_ 的状态，不受膨胀影响
            //   - stable_cells_[idx].state == -1 -> 稳定 unknown
            //   - stable_cells_[idx].state == 0  -> 稳定 free
            //   - stable_cells_[idx].state == 100-> 稳定 occupied
            if (stable_cells_[idx].state == -1) {
                ++unknown;
            }
        }
    }

    if (total == 0) {
        // 极端情况：没统计到任何栅格，就认为全未知
        return 1.0f;
    }

    float ratio = static_cast<float>(unknown) / static_cast<float>(total);

    // 可选：给 unknown_ratio 设置一个上限，避免一片 unknown 把置信度拉到过低
    // ratio = std::min(ratio, 0.8f);

    return ratio;
}


std::vector<Eigen::Vector2i> PathPlanner::resamplePath(const std::vector<Eigen::Vector2i>& path_indices,
                                                       double step_m) const {
    if (path_indices.size() <= 2 || step_m <= 1e-3) return path_indices;

    std::vector<Eigen::Vector2i> out;
    out.reserve(path_indices.size());
    out.push_back(path_indices.front());

    double acc = 0.0;
    geometry_msgs::Point prev = gridToWorld(path_indices.front());

    for (size_t i = 1; i < path_indices.size(); ++i) {
        geometry_msgs::Point cur = gridToWorld(path_indices[i]);
        const double seg = std::hypot(cur.x - prev.x, cur.y - prev.y);
        acc += seg;
        if (acc >= step_m) {
            out.push_back(path_indices[i]);
            acc = 0.0;
            prev = cur;
        }
    }

    if (out.back().x() != path_indices.back().x() || out.back().y() != path_indices.back().y()) {
        out.push_back(path_indices.back());
    }
    return out;
}



// ===================== 智能安全重采样相关函数实现 =====================

Eigen::Vector2i PathPlanner::interpolateOnGrid(const Eigen::Vector2i& start,
                                               const Eigen::Vector2i& end, double t) const {
    // 在栅格坐标之间进行线性插值
    double x = start.x() + t * (end.x() - start.x());
    double y = start.y() + t * (end.y() - start.y());

    // 四舍五入到最近的栅格
    return Eigen::Vector2i(static_cast<int>(std::round(x)),
                          static_cast<int>(std::round(y)));
}

std::vector<Eigen::Vector2i> PathPlanner::localReplan(const Eigen::Vector2i& start_pos,
                                                      const Eigen::Vector2i& end_pos) const {
    if (!grid_info_) return {};

    // 创建局部障碍物掩码
    std::vector<bool> blocked_mask = createBlockedMask(start_pos, end_pos);

    // 使用现有的findPath函数进行局部规划
    // 由于findPath不是const函数，我们需要const_cast
    auto* planner = const_cast<PathPlanner*>(this);
    std::vector<Eigen::Vector2i> local_path = planner->findPath(start_pos, end_pos, &blocked_mask);

    if (local_path.empty()) {
        // 回退策略：尝试更简单的直线路径
        ROS_WARN("局部重规划失败，尝试直线路径");
        if (isLineFree(start_pos, end_pos)) {
            return {start_pos, end_pos};
        }
    }

    return local_path;
}



std::vector<bool> PathPlanner::createBlockedMask(const Eigen::Vector2i& start,
                                                 const Eigen::Vector2i& end) const {
    if (!grid_info_) return {};

    std::vector<bool> blocked_mask(grid_info_->info.width * grid_info_->info.height, false);

    // 计算搜索区域的边界
    int min_x = std::max(0, std::min(start.x(), end.x()) - static_cast<int>(safe_resample_search_radius_ / grid_info_->info.resolution));
    int max_x = std::min(static_cast<int>(grid_info_->info.width) - 1,
                         std::max(start.x(), end.x()) + static_cast<int>(safe_resample_search_radius_ / grid_info_->info.resolution));
    int min_y = std::max(0, std::min(start.y(), end.y()) - static_cast<int>(safe_resample_search_radius_ / grid_info_->info.resolution));
    int max_y = std::min(static_cast<int>(grid_info_->info.height) - 1,
                         std::max(start.y(), end.y()) + static_cast<int>(safe_resample_search_radius_ / grid_info_->info.resolution));

    // 标记已知障碍物区域
    for (int y = min_y; y <= max_y; ++y) {
        for (int x = min_x; x <= max_x; ++x) {
            Eigen::Vector2i pos(x, y);
            if (isObstacle(pos)) {
                int idx = y * grid_info_->info.width + x;
                blocked_mask[idx] = true;
            }
        }
    }

    return blocked_mask;
}



size_t PathPlanner::findClosestIndex(const std::vector<Eigen::Vector2i>& path,
                                     const Eigen::Vector2i& target_pos) const {
    if (path.empty()) return 0;

    size_t closest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < path.size(); ++i) {
        double dist = std::hypot(path[i].x() - target_pos.x(),
                                path[i].y() - target_pos.y());
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }

    return closest_idx;
}



Eigen::Vector2i PathPlanner::findNextSafePoint(size_t start_idx) const {
    if (!grid_info_) return Eigen::Vector2i(-1, -1);

    // 从start_idx开始向后寻找下一个安全的点
    const int max_search_distance = static_cast<int>(safe_resample_search_radius_ / grid_info_->info.resolution * 2);

    for (int dist = 1; dist <= max_search_distance; ++dist) {
        // 在当前点周围搜索安全点
        for (int dx = -dist; dx <= dist; ++dx) {
            for (int dy = -dist; dy <= dist; ++dy) {
                // 只搜索边界上的点
                if (std::abs(dx) != dist && std::abs(dy) != dist) continue;

                // 修复：基于搜索半径创建安全点，而不是基于未定义的start_pos
                int x = static_cast<int>(safe_resample_search_radius_ / grid_info_->info.resolution) + dx;
                int y = static_cast<int>(safe_resample_search_radius_ / grid_info_->info.resolution) + dy;

                if (x >= 0 && x < static_cast<int>(grid_info_->info.width) &&
                    y >= 0 && y < static_cast<int>(grid_info_->info.height)) {
                    Eigen::Vector2i pos(x, y);
                    if (!isObstacle(pos) && isCorridorSafeAtPosition(pos, 0.5)) {
                        return pos;
                    }
                }
            }
        }
    }

    // 如果找不到安全点，返回一个远离障碍物的点
    int safe_x = static_cast<int>(start_idx + safe_resample_search_radius_);
    int safe_y = static_cast<int>(start_idx + safe_resample_search_radius_);
    safe_x = std::max(0, std::min(safe_x, static_cast<int>(grid_info_->info.width) - 1));
    safe_y = std::max(0, std::min(safe_y, static_cast<int>(grid_info_->info.height) - 1));
    return Eigen::Vector2i(safe_x, safe_y);
}

double PathPlanner::calculateEnvironmentComplexity(const Eigen::Vector2i& pos) const {
    if (!grid_info_) return 0.5; // 默认中等复杂度

    const int radius = static_cast<int>(safe_resample_search_radius_ / grid_info_->info.resolution);
    int obstacle_count = 0;
    int total_count = 0;

    // 统计周围区域的障碍物密度
    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            int x = pos.x() + dx;
            int y = pos.y() + dy;

            if (x >= 0 && x < static_cast<int>(grid_info_->info.width) &&
                y >= 0 && y < static_cast<int>(grid_info_->info.height)) {
                total_count++;
                if (isObstacle(Eigen::Vector2i(x, y))) {
                    obstacle_count++;
                }
            }
        }
    }

    if (total_count == 0) return 0.5;

    // 返回0-1之间的复杂度分数
    return static_cast<double>(obstacle_count) / total_count;
}



double PathPlanner::adaptiveStepAdjustment(const Eigen::Vector2i& pos, double base_step_m) const {
    if (!adaptive_density_enable_) {
        return base_step_m;
    }

    double complexity = calculateEnvironmentComplexity(pos);

    if (complexity > high_complexity_threshold_) {
        // 高复杂度区域：使用更小的步长增加采样密度
        return base_step_m * 0.5;
    } else if (complexity < low_complexity_threshold_) {
        // 低复杂度区域：使用更大的步长减少采样密度
        return base_step_m * 1.5;
    } else {
        // 中等复杂度：使用标准步长
        return base_step_m;
    }
}

std::vector<Eigen::Vector2i> PathPlanner::intelligentSafeResample(
    const std::vector<Eigen::Vector2i>& path_indices,
    double base_step_m) const {

    if (path_indices.size() <= 2 || base_step_m <= 1e-3) {
        return path_indices; // 保持与原函数兼容
    }

    std::vector<Eigen::Vector2i> result_path;
    result_path.reserve(path_indices.size() * 2); // 预留更多空间

    result_path.push_back(path_indices.front());

    size_t current_idx = 0;
    Eigen::Vector2i current_pos = path_indices[current_idx];

    while (current_idx < path_indices.size() - 1) {
        // 寻找目标距离点
        double accumulated_dist = 0.0;
        bool found_next_point = false;

        // 找到距离至少为base_step_m的下一个原始路径点
        for (size_t i = current_idx + 1; i < path_indices.size(); ++i) {
            geometry_msgs::Point world_current = gridToWorld(current_pos);
            geometry_msgs::Point world_next = gridToWorld(path_indices[i]);
            double segment_dist = std::hypot(world_next.x - world_current.x,
                                           world_next.y - world_current.y);

            if (accumulated_dist + segment_dist >= base_step_m) {
                // 在此段内进行插值采样
                double remaining_dist = base_step_m - accumulated_dist;
                double t = remaining_dist / segment_dist;

                Eigen::Vector2i interpolated_pos = interpolateOnGrid(
                    path_indices[i-1], path_indices[i], t);

                // 自适应步长调整
                double adaptive_step = adaptive_density_enable_ ?
                    adaptiveStepAdjustment(interpolated_pos, base_step_m) : base_step_m;

                // 安全性检查 - 使用默认1m走廊宽度检查
                if (isCorridorSafeAtPosition(interpolated_pos, 0.5)) {
                    result_path.push_back(interpolated_pos);
                    current_pos = interpolated_pos;
                    current_idx = i - 1; // 从插值点继续向前搜索
                    found_next_point = true;
                    break;
                } else {
                    // 触发局部重规划
                    ROS_DEBUG("发现不安全位置，触发局部重规划");
                    // 简单处理：跳过这个不安全点，继续寻找下一个安全点
                    found_next_point = false; // 继续循环寻找下一个点
                }
            }

            accumulated_dist += segment_dist;

            // 如果到达终点但距离仍不足，跳出循环
            if (accumulated_dist < base_step_m && i == path_indices.size() - 1) {
                found_next_point = true;
                break;
            }
        }

        if (!found_next_point) {
            break;
        }
    }

    // 确保包含终点
    if (!result_path.empty() &&
        (result_path.back() != path_indices.back() ||
         std::hypot(gridToWorld(result_path.back()).x - gridToWorld(path_indices.back()).x,
                   gridToWorld(result_path.back()).y - gridToWorld(path_indices.back()).y) > base_step_m * 0.5)) {
        result_path.push_back(path_indices.back());
    }

    return result_path;
}



std::vector<Eigen::Vector2i> PathPlanner::removePathLoops(
    const std::vector<Eigen::Vector2i>& path) const
{
    if (!grid_info_ || path.size() <= 2) {
        return path;
    }

    const int W = grid_info_->info.width;

    std::vector<Eigen::Vector2i> result;
    result.reserve(path.size());

    // 记录某个"格子索引"第一次出现在 result 里的下标
    std::unordered_map<int, size_t> first_seen;

    for (size_t i = 0; i < path.size(); ++i) {
        const auto& p = path[i];
        int idx = p.y() * W + p.x();   // 把 (x,y) 编成一个整型索引

        auto it = first_seen.find(idx);
        if (it == first_seen.end()) {
            // 第一次看到这个格子，正常加入
            first_seen[idx] = result.size();
            result.push_back(p);
        } else {
            // 发现环：result[ it->second .. result.size()-1 ] 这一段是回路
            size_t s = it->second;

            // 保留 [0..s]，丢掉后面所有点
            result.resize(s + 1);

            // 由于 result 变短了，first_seen 里原来的下标已经不对了，
            // 直接重新构建一遍 first_seen（路径长度一般不大，这点开销可以接受）
            first_seen.clear();
            for (size_t k = 0; k < result.size(); ++k) {
                int idx_k = result[k].y() * W + result[k].x();
                first_seen[idx_k] = k;
            }

            // 当前这个点等于 result[s]，已经在 result 末尾了，不需要再插一次
        }
    }

    return result;
}

} // namespace fitplane_planner
