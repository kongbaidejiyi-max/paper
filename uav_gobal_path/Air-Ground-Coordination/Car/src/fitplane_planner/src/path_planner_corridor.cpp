#include "fitplane_planner/path_planner.h"
#include <algorithm>

namespace fitplane_planner 
{

double PathPlanner::getActualCorridorWidth(const geometry_msgs::Point& position) const {
    // 获取指定位置的实际corridor全宽
    Eigen::Vector2i g = worldToGrid(position);

    if (g.x() < 0 || g.x() >= grid_info_->info.width || g.y() < 0 || g.y() >= grid_info_->info.height) {
        return 0.0;
    }

    const int idx = g.y() * grid_info_->info.width + g.x();
    if (idx < 0 || idx >= static_cast<int>(distance_to_obstacle_.size())) {
        return 0.0;
    }

    const float dcell = distance_to_obstacle_[idx];

    if (std::isfinite(dcell)) {
        // 基于实际距离场计算corridor宽度
        double clearance_m = dcell * grid_info_->info.resolution - corridor_margin_;
        return std::max(0.0, clearance_m * 2);  // 返回全宽
    } else {
        // 未知区域使用保守宽度
        return std::min(corridor_min_width_, corridor_max_width_);
    }
}



double PathPlanner::getSafeCorridorWidth(const geometry_msgs::Point& position, double initial_half_width,
                                        double min_half_width, double max_half_width) const {
    // Enhanced conservative corridor width adjustment based on obstacle detection

    // Always prioritize safety over expansion
    if (!doesCorridorIntersectObstacle(position, initial_half_width)) {
        // Initial width is safe, but be very conservative about expansion
        double safe_half_width = initial_half_width;

        // Only try modest expansion, not aggressive
        double low = initial_half_width;
        double high = std::min(max_half_width, initial_half_width * 1.2); // Much more conservative expansion

        for (int iter = 0; iter < 8; ++iter) { // Fewer iterations for performance
            if (high - low < 0.05) break; // Tighter precision threshold

            double mid = (low + high) / 2.0;

            if (!doesCorridorIntersectObstacle(position, mid)) {
                safe_half_width = mid;
                low = mid;
            } else {
                high = mid;
                break; // Immediately stop if we hit an obstacle
            }
        }

        return safe_half_width;
    } else {
        // Initial width intersects obstacles, be more aggressive in shrinking
        double safe_half_width = min_half_width; // Start with minimum

        // Binary search for safe width from bottom up
        double low = min_half_width;
        double high = initial_half_width;

        for (int iter = 0; iter < 12; ++iter) { // More iterations for precision
            if (high - low < 0.02) break; // Much tighter precision for shrinking

            double mid = (low + high) / 2.0;

            if (!doesCorridorIntersectObstacle(position, mid)) {
                safe_half_width = mid;
                low = mid;
            } else {
                high = mid;
            }
        }

        return safe_half_width;
    }
}



double PathPlanner::calculateOptimalCorridorWidth(const geometry_msgs::Point& position) const {
    // Calculate the optimal corridor width that prioritizes safety above all

    // Get the base corridor width from the existing method
    double base_full_width = getActualCorridorWidth(position);
    double base_half_width = base_full_width / 2.0;

    // Apply conservative safety limits
    double min_half_width = 0.15;  // Minimum 0.3m total width (more conservative)
    double max_half_width = std::min(2.5, corridor_max_width_ / 2.0);  // More conservative maximum

    double safe_half_width = getSafeCorridorWidth(position, base_half_width, min_half_width, max_half_width);

    // Apply additional safety margin - corridors should be more conservative in complex areas
    safe_half_width = std::max(min_half_width, std::min(safe_half_width, max_half_width));

    // Return the safe full width WITHOUT expansion - safety is priority
    return safe_half_width * 2.0; // Return full width (half_width * 2), no expansion factor
}

bool PathPlanner::isCorridorStillSafe(double safety_margin_m) const
{
    if (!grid_info_ || last_published_path_.poses.empty()) return false;

    const int    W   = grid_info_->info.width;
    const int    H   = grid_info_->info.height;
    const double res = grid_info_->info.resolution;

    size_t check_count = 0;
    for (const auto& ps : last_published_path_.poses) {
        Eigen::Vector2i g = worldToGrid(ps.pose.position);

        if (g.x() < 0 || g.x() >= W || g.y() < 0 || g.y() >= H) {
            // 越界直接判不安全
            ROS_WARN("[CORRIDOR_CHECK] Path point %zu at (%d,%d) out of bounds",
                     check_count, g.x(), g.y());
            return false;
        }

        if (getCost(g) < 0.0) {
            // 路径中心落在致命障碍物上
            ROS_WARN("[CORRIDOR_CHECK] Path point %zu at (%d,%d) center on obstacle",
                     check_count, g.x(), g.y());
            return false;
        }

        // 获取安全的corridor半宽（考虑障碍物占据检测）
        double actual_corridor_half_width = calculateOptimalCorridorWidth(ps.pose.position) / 2.0;

        // 修复：增加采样密度，从8个增加到16个点
        const int boundary_samples = 16;
        double theta = tf::getYaw(ps.pose.orientation);

        for (int side = -1; side <= 1; side += 2) {
            for (int i = 1; i <= boundary_samples; ++i) {
                double sample_offset = (actual_corridor_half_width * i) / boundary_samples;

                // 使用高精度边界检查
                double boundary_cost = getCostAtOffsetPrecise(g.x(), g.y(), theta, sample_offset * side);

                if (boundary_cost < 0) {
                    ROS_WARN("[CORRIDOR_CHECK] Path point %zu at (%d,%d) offset %.2f hits obstacle (hw=%.2f)",
                             check_count, g.x(), g.y(), sample_offset * side, actual_corridor_half_width);
                    return false;
                }

                // 额外的亚米级检查，防止遗漏小障碍物
                for (double extra_offset : {0.05, 0.1, 0.15}) {
                    double extra_cost = getCostAtOffsetPrecise(g.x(), g.y(), theta,
                                                     sample_offset * side + extra_offset * side);
                    if (extra_cost < 0) {
                        ROS_WARN("[CORRIDOR_CHECK] Path point %zu at (%d,%d) extended offset %.2f hits obstacle",
                                 check_count, g.x(), g.y(), sample_offset * side + extra_offset * side);
                        return false;
                    }
                }
            }
        }
        check_count++;
    }
    ROS_INFO("[CORRIDOR_CHECK] All %zu path points passed corridor safety check (margin=%.2fm)",
             check_count, safety_margin_m);
    return true;
}



// Corridor obstacle detection and dynamic adjustment functions
bool PathPlanner::doesCorridorIntersectObstacle(const geometry_msgs::Point& position, double half_width, int num_samples) const {
    // Enhanced conservative corridor obstacle detection with safety margin

    // Add safety margin to make detection more conservative
    double check_radius = half_width * 1.1; // 10% safety margin
    int enhanced_samples = std::max(24, num_samples); // Use more samples for better coverage

    // Check boundary points with safety margin
    for (int i = 0; i < enhanced_samples; ++i) {
        double angle = 2.0 * M_PI * i / enhanced_samples;

        // Calculate sample point position with safety margin
        double check_x = position.x + check_radius * std::cos(angle);
        double check_y = position.y + check_radius * std::sin(angle);

        geometry_msgs::Point check_point;
        check_point.x = check_x;
        check_point.y = check_y;
        check_point.z = 0.0;

        Eigen::Vector2i check_pos = worldToGrid(check_point);

        if (isObstacle(check_pos)) {
            return true; // Found obstacle intersection
        }
    }

    // Additional conservative check: sample interior points as well
    int interior_samples = 8;
    for (int i = 0; i < interior_samples; ++i) {
        double angle = 2.0 * M_PI * i / interior_samples;
        double interior_radius = half_width * 0.7; // Check at 70% of radius

        double check_x = position.x + interior_radius * std::cos(angle);
        double check_y = position.y + interior_radius * std::sin(angle);

        geometry_msgs::Point check_point;
        check_point.x = check_x;
        check_point.y = check_y;
        check_point.z = 0.0;

        Eigen::Vector2i check_pos = worldToGrid(check_point);

        if (isObstacle(check_pos)) {
            return true; // Found obstacle intersection in interior
        }
    }

    return false; // No obstacle intersection found
}



bool PathPlanner::hasObstacleInCorridor(const Eigen::Vector2i& center_pos, double half_width) const {
    if (!grid_info_) return false;

    geometry_msgs::Point world_center = gridToWorld(center_pos);

    // 在走廊圆周上进行采样检查
    const int num_samples = 16;
    for (int i = 0; i < num_samples; ++i) {
        double angle = 2.0 * M_PI * i / num_samples;
        double check_x = world_center.x + half_width * std::cos(angle);
        double check_y = world_center.y + half_width * std::sin(angle);

        geometry_msgs::Point check_point;
        check_point.x = check_x;
        check_point.y = check_y;
        check_point.z = 0.0;
        Eigen::Vector2i check_pos = worldToGrid(check_point);

        // 检查是否超出地图边界
        if (check_pos.x() < 0 || check_pos.x() >= static_cast<int>(grid_info_->info.width) ||
            check_pos.y() < 0 || check_pos.y() >= static_cast<int>(grid_info_->info.height)) {
            return true; // 超出边界视为不安全
        }

        if (isObstacle(check_pos)) {
            return true; // 发现障碍物
        }
    }

    return false; // 无障碍物
}



bool PathPlanner::isCorridorSafeAtPosition(const Eigen::Vector2i& pos, double half_width) const {
    // 首先检查位置本身是否有效
    if (pos.x() < 0 || pos.x() >= static_cast<int>(grid_info_->info.width) ||
        pos.y() < 0 || pos.y() >= static_cast<int>(grid_info_->info.height)) {
        return false;
    }

    // 检查位置本身是否为障碍物
    if (isObstacle(pos)) {
        return false;
    }

    // 使用实际计算的走廊宽度进行检查
    double actual_half_width = getActualCorridorWidth(gridToWorld(pos)) / 2.0;
    double check_width = std::min(half_width, actual_half_width);

    if (check_width <= 0.1) { // 走廊宽度过小
        return false;
    }

    // 检查走廊是否与障碍物相交
    return !hasObstacleInCorridor(pos, check_width);
}

} // namespace fitplane_planner
