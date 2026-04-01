#include "fitplane_planner/path_planner.h"
#include <fitplane_planner/GlobalPath.h>   // ★ 新增
#include <algorithm>
#include <chrono>
#include <cmath>
#include <tf/transform_datatypes.h>

namespace fitplane_planner 
{

namespace {
inline double msSince(std::chrono::steady_clock::time_point start) {
    using namespace std::chrono;
    return duration_cast<duration<double, std::milli>>(steady_clock::now() - start).count();
}
} // namespace

void PathPlanner::publishAstarRawMarker(const std::vector<Eigen::Vector2i>& path_indices) {
    if (!astar_raw_marker_enable_) return;
    if (pub_astar_raw_marker_.getNumSubscribers() == 0) return;

    visualization_msgs::Marker m;
    m.header.frame_id = world_frame_;
    m.header.stamp = ros::Time::now();
    m.ns = "astar_raw";
    m.id = 0;
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::POINTS;
    m.pose.orientation.w = 1.0;

    const double pt = (astar_raw_marker_point_size_ > 1e-6) ? astar_raw_marker_point_size_
                                                            : std::max(0.02, 0.25 * grid_info_->info.resolution);
    m.scale.x = pt;
    m.scale.y = pt;
    m.color.r = 1.0;
    m.color.g = 0.15;
    m.color.b = 0.15;
    m.color.a = 1.0;

    if (path_indices.empty()) {
        m.action = visualization_msgs::Marker::DELETE;
        pub_astar_raw_marker_.publish(m);
        return;
    }

    const size_t n = path_indices.size();
    size_t stride = 1;
    if (astar_raw_marker_max_points_ > 0 && n > static_cast<size_t>(astar_raw_marker_max_points_)) {
        stride = (n + static_cast<size_t>(astar_raw_marker_max_points_) - 1) /
                 static_cast<size_t>(astar_raw_marker_max_points_);
        if (stride < 1) stride = 1;
    }

    m.points.reserve((n + stride - 1) / stride);
    for (size_t i = 0; i < n; i += stride) {
        geometry_msgs::Point p = gridToWorld(path_indices[i]);
        p.z = astar_raw_marker_z_;
        m.points.push_back(p);
    }
    pub_astar_raw_marker_.publish(m);
}

void PathPlanner::publishGrpCenterlineMarker(const std::vector<geometry_msgs::Pose>& poses) {
    if (!grp_centerline_marker_enable_) return;
    if (pub_grp_centerline_marker_.getNumSubscribers() == 0) return;

    visualization_msgs::Marker m;
    m.header.frame_id = world_frame_;
    m.header.stamp = ros::Time::now();
    m.ns = "grp_centerline";
    m.id = 0;
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.pose.orientation.w = 1.0;
    m.scale.x = std::max(0.001, grp_centerline_marker_width_);
    m.color.r = 0.1;
    m.color.g = 0.9;
    m.color.b = 1.0;
    m.color.a = 1.0;

    if (poses.empty()) {
        m.action = visualization_msgs::Marker::DELETE;
        pub_grp_centerline_marker_.publish(m);
        return;
    }

    m.points.reserve(poses.size());
    for (const auto& ps : poses) {
        geometry_msgs::Point p = ps.position;
        p.z = grp_centerline_marker_z_;
        m.points.push_back(p);
    }
    pub_grp_centerline_marker_.publish(m);
}

void PathPlanner::addTriangleToMarker(visualization_msgs::Marker& marker,
                           const geometry_msgs::Point& p1,
                           const geometry_msgs::Point& p2,
                           const geometry_msgs::Point& p3,
                           const std_msgs::ColorRGBA& color) {
    marker.points.push_back(p1);
    marker.points.push_back(p2);
    marker.points.push_back(p3);
    marker.colors.push_back(color);
    marker.colors.push_back(color);
    marker.colors.push_back(color);
}



void PathPlanner::publishPathVisualization(const std::vector<Eigen::Vector2i>& path_indices, const std::vector<float>& half_width) {
    (void)half_width;
    if (pub_path_marker_.getNumSubscribers() == 0) {
        return;
    }
       // 先删除旧的 marker
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = world_frame_;
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.ns = "path";
    delete_marker.id = 0;
    delete_marker.action = visualization_msgs::Marker::DELETE;
    pub_path_marker_.publish(delete_marker);


    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = world_frame_;
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "path";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 0;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    // 设置线条宽度和颜色
    line_strip.scale.x = path_marker_width_;  // 路径线条宽度
    line_strip.color.g = 1.0;  // 绿色
    line_strip.color.a = 1.0;  // 透明度为 1

    // 将路径点添加到 Marker 中
    for (const auto& idx : path_indices) {
        geometry_msgs::Point p;
        p = gridToWorld(idx);  // 将栅格坐标转换为世界坐标
        line_strip.points.push_back(p);
    }

    pub_path_marker_.publish(line_strip);  // 发布路径可视化
}



void PathPlanner::publishConfidenceVisualization(const std::vector<Eigen::Vector2i>& path_indices,
    const std::vector<float>& confidence,
    const std::vector<float>& half_width) {
    if (pub_corridor_marker_.getNumSubscribers() == 0) {
        return;
    }

    const auto t_start = std::chrono::steady_clock::now();
    const size_t n_path = path_indices.size();
    const size_t n_conf = confidence.size();
    const size_t n_hw   = half_width.size();
    const size_t n_in = std::min(n_path, std::min(n_conf, n_hw));

    auto publishDelete = [&](int id) {
        visualization_msgs::Marker m;
        m.header.frame_id = world_frame_;
        m.header.stamp = ros::Time::now();
        m.ns = "corridors";
        m.id = id;
        m.action = visualization_msgs::Marker::DELETE;
        pub_corridor_marker_.publish(m);
    };

    if (n_in < 2) {
        publishDelete(1);
        publishDelete(2);
        publishDelete(3);
        publishDelete(4);
        return;
    }

    // 可视化可能只取子采样，避免 RViz Marker 过大
    std::vector<size_t> keep;
    keep.reserve(n_in);
    keep.push_back(0);
    if (corridor_viz_max_segments_ > 0 && (n_in - 1) > static_cast<size_t>(corridor_viz_max_segments_)) {
        const size_t max_seg = static_cast<size_t>(corridor_viz_max_segments_);
        const size_t stride = (n_in - 1 + max_seg - 1) / max_seg;
        for (size_t i = stride; i + 1 < n_in; i += stride) {
            keep.push_back(i);
        }
    } else {
        for (size_t i = 1; i + 1 < n_in; ++i) {
            keep.push_back(i);
        }
    }
    keep.push_back(n_in - 1);

    const size_t n = keep.size();
    if (n < 2) {
        publishDelete(1);
        publishDelete(2);
        publishDelete(3);
        publishDelete(4);
        return;
    }

    const bool color_by_time = (corridor_viz_color_mode_ == "time");
    auto valueAt = [&](size_t i) -> float {
        if (color_by_time) {
            if (n <= 1) return 0.0f;
            return static_cast<float>(i) / static_cast<float>(n - 1);
        }
        return clampf(confidence[keep[i]], 0.0f, 1.0f);
    };

    auto riskFillColor = [&](float v, float alpha) {
        const float t = clampf(v, 0.0f, 1.0f);
        // 内部填充：不安全(0) -> 红色, 安全(1) -> 绿色
        std_msgs::ColorRGBA c;
        c.r = 1.0f - t;
        c.g = t;
        c.b = 0.0f;
        c.a = clampf(alpha, 0.0f, 1.0f);
        return c;
    };

    auto tealLineColor = [&](float v, float alpha, float dark_scale) {
        const float t = clampf(v, 0.0f, 1.0f);
        // 线条：蓝 -> 青绿（用于边界/网格）
        float r = (1.0f - t) * 0.10f + t * 0.00f;
        float g = (1.0f - t) * 0.55f + t * 0.95f;
        float b = (1.0f - t) * 1.00f + t * 0.70f;
        r *= dark_scale;
        g *= dark_scale;
        b *= dark_scale;
        std_msgs::ColorRGBA c;
        c.r = clampf(r, 0.0f, 1.0f);
        c.g = clampf(g, 0.0f, 1.0f);
        c.b = clampf(b, 0.0f, 1.0f);
        c.a = clampf(alpha, 0.0f, 1.0f);
        return c;
    };

    const double z_bottom_fill = corridor_viz_z_;
    const double z_top_fill = corridor_viz_z_ + std::max(0.0, corridor_viz_height_);
    const double z_bottom_outline = corridor_viz_z_ + corridor_viz_z_outline_offset_;
    const double z_top_outline = z_bottom_outline + std::max(0.0, corridor_viz_height_);
    const double z_bottom_grid = corridor_viz_z_ + corridor_viz_z_grid_offset_;
    const double z_top_grid = z_bottom_grid + std::max(0.0, corridor_viz_height_);
    const double z_centerline = corridor_viz_z_ + 0.5 * std::max(0.0, corridor_viz_height_) + corridor_viz_z_centerline_offset_;
    const double miter_limit = std::max(1.0, corridor_viz_miter_limit_);

    // 预计算中心点（world）与每段法向量（2D）
    std::vector<geometry_msgs::Point> center(n);
    for (size_t i = 0; i < n; ++i) {
        center[i] = gridToWorld(path_indices[keep[i]]);
    }

    const double eps = 1e-9;
    struct Vec2 { double x; double y; };
    auto norm2 = [&](const Vec2& v) { return std::hypot(v.x, v.y); };
    auto normalize = [&](const Vec2& v) {
        const double l = norm2(v);
        if (l < eps) return Vec2{0.0, 0.0};
        return Vec2{v.x / l, v.y / l};
    };

    std::vector<Vec2> seg_n(n - 1, {0.0, 0.0});
    for (size_t i = 0; i + 1 < n; ++i) {
        const double dx = center[i + 1].x - center[i].x;
        const double dy = center[i + 1].y - center[i].y;
        const double l = std::hypot(dx, dy);
        if (l < 1e-6) continue;
        seg_n[i] = Vec2{-dy / l, dx / l};
    }

    size_t first_valid = 0;
    while (first_valid + 1 < n && norm2(seg_n[first_valid]) < 1e-6) ++first_valid;
    size_t last_valid = n - 2;
    while (last_valid > 0 && norm2(seg_n[last_valid]) < 1e-6) --last_valid;
    if (norm2(seg_n[first_valid]) < 1e-6 || norm2(seg_n[last_valid]) < 1e-6) {
        publishDelete(1);
        publishDelete(2);
        publishDelete(3);
        publishDelete(4);
        return;
    }

    // 非对称走廊：对每个中心点估计左右单侧清空度（沿法向/偏移方向），并在可行时维持名义宽度
    const double hw_max = 0.5 * corridor_max_width_;
    const double nominal_full = (corridor_viz_nominal_width_ > 1e-6) ? corridor_viz_nominal_width_ : corridor_min_width_;
    const double base_target_full_width = std::max(0.0, std::min(nominal_full, corridor_max_width_));
    const double step_m = std::max(0.01, corridor_viz_clearance_step_m_);
    const double cost_thresh = corridor_viz_obstacle_cost_threshold_;
    const double unknown_cost = unknown_space_cost_;

    auto isBlockedCost = [&](double cost) {
        if (cost < 0.0) return true;
        if (corridor_viz_unknown_is_obstacle_ && std::abs(cost - unknown_cost) < 1e-6) return true;
        if (std::isfinite(cost_thresh) && cost_thresh > 0.0 && cost >= cost_thresh) return true;
        return false;
    };

    auto estimateSideClearance = [&](const geometry_msgs::Point& p_center, double theta, int sign) -> double {
        // sign: +1 左侧（+offset）, -1 右侧（-offset）
        const Eigen::Vector2i g = worldToGrid(p_center);
        double last_ok = 0.0;
        for (double d = step_m; d <= hw_max + 1e-9; d += step_m) {
            const double c = getCostAtOffsetPrecise(g.x(), g.y(), theta, sign * d);
            if (isBlockedCost(c)) break;
            last_ok = d;
        }
        // corridor_margin_：额外安全边（可视化也反映）
        return std::max(0.0, last_ok - corridor_margin_);
    };

    std::vector<double> hw_left(n, 0.0), hw_right(n, 0.0);
    for (size_t i = 0; i < n; ++i) {
        // 用邻点估计切向方向（theta），保证与 getCostAtOffsetPrecise 的 offset 定义一致
        const geometry_msgs::Point& p = center[i];
        geometry_msgs::Point p_prev = (i > 0) ? center[i - 1] : center[i];
        geometry_msgs::Point p_next = (i + 1 < n) ? center[i + 1] : center[i];
        const double dx = p_next.x - p_prev.x;
        const double dy = p_next.y - p_prev.y;
        double theta = std::atan2(dy, dx);
        if (std::hypot(dx, dy) < 1e-6) {
            // 退化：用全局固定角度，至少不崩
            theta = 0.0;
        }

        double left_clear = estimateSideClearance(p, theta, +1);
        double right_clear = estimateSideClearance(p, theta, -1);

        left_clear = std::max(0.0, std::min(left_clear, hw_max));
        right_clear = std::max(0.0, std::min(right_clear, hw_max));

        // 已知区域：允许在不碰撞的前提下适度增宽（仅影响可视化，GRP 半宽仍由 computeCorridorAndConfidence 决定）
        double target_full_width = base_target_full_width;
        {
            const Eigen::Vector2i g = worldToGrid(p);
            const double unknown_ratio = computeUnknownRatio(g, conf_local_radius_cells_);
            if (unknown_ratio <= corridor_viz_known_unknown_thresh_ && corridor_viz_known_expand_ratio_ > 1.0) {
                target_full_width = std::min(corridor_max_width_, base_target_full_width * corridor_viz_known_expand_ratio_);
            }
        }
        const double half_target = 0.5 * target_full_width;

        // 默认：两侧都够宽时保持标准矩形（避免没必要的偏心）
        if (!corridor_viz_asym_enable_ || (left_clear >= half_target && right_clear >= half_target)) {
            hw_left[i] = std::min(half_target, left_clear);
            hw_right[i] = std::min(half_target, right_clear);
            continue;
        }

        // 非对称：按单侧可用空间分配宽度，尽量维持 target_full_width（不行则缩到可用总宽）
        const double avail = left_clear + right_clear;
        if (avail < 1e-6) {
            hw_left[i] = 0.0;
            hw_right[i] = 0.0;
            continue;
        }

        const double tgt = std::min(target_full_width, avail);
        double wl = tgt * (left_clear / avail);
        double wr = tgt - wl;

        wl = std::min(wl, left_clear);
        wr = std::min(wr, right_clear);

        // 如果还没达到 tgt，尽量把剩余补到更“宽”的那一侧
        double rem = tgt - (wl + wr);
        if (rem > 1e-6) {
            const double room_r = right_clear - wr;
            const double add_r = std::min(rem, room_r);
            wr += add_r;
            rem -= add_r;

            const double room_l = left_clear - wl;
            const double add_l = std::min(rem, room_l);
            wl += add_l;
        }

        hw_left[i] = wl;
        hw_right[i] = wr;
    }

    // 计算每个顶点的 offset（2D miter，让拐角更顺滑），然后扩成 3D box 截面
    std::vector<geometry_msgs::Point> lb(n), rb(n), lt(n), rt(n);
    for (size_t i = 0; i < n; ++i) {
        const Vec2 n_prev = (i == 0) ? seg_n[first_valid] : seg_n[std::min(i - 1, n - 2)];
        const Vec2 n_next = (i + 1 >= n) ? seg_n[last_valid] : seg_n[std::min(i, n - 2)];
        const bool prev_ok = norm2(n_prev) > 1e-6;
        const bool next_ok = norm2(n_next) > 1e-6;

        Vec2 bisector{0.0, 0.0};
        if (prev_ok && next_ok) {
            bisector = normalize(Vec2{n_prev.x + n_next.x, n_prev.y + n_next.y});
            if (norm2(bisector) < 1e-6) {
                bisector = n_next;
            }
        } else if (next_ok) {
            bisector = n_next;
        } else if (prev_ok) {
            bisector = n_prev;
        } else {
            bisector = seg_n[first_valid];
        }

        auto miterScale = [&](double hw) -> double {
            double scale = hw;
            if (prev_ok && next_ok && norm2(bisector) > 1e-6) {
                const double denom = std::abs(bisector.x * n_prev.x + bisector.y * n_prev.y);
                if (denom > 1e-3) {
                    scale = hw / denom;
                    const double max_scale = miter_limit * hw;
                    scale = std::max(0.0, std::min(max_scale, scale));
                }
            }
            return scale;
        };

        const double scale_l = miterScale(hw_left[i]);
        const double scale_r = miterScale(hw_right[i]);

        lb[i].x = center[i].x + bisector.x * scale_l;
        lb[i].y = center[i].y + bisector.y * scale_l;
        lb[i].z = z_bottom_fill;

        rb[i].x = center[i].x - bisector.x * scale_r;
        rb[i].y = center[i].y - bisector.y * scale_r;
        rb[i].z = z_bottom_fill;

        lt[i] = lb[i];
        rt[i] = rb[i];
        lt[i].z = z_top_fill;
        rt[i].z = z_top_fill;
    }

    auto lerpPoint = [](const geometry_msgs::Point& a, const geometry_msgs::Point& b, double t) {
        geometry_msgs::Point p;
        p.x = a.x + (b.x - a.x) * t;
        p.y = a.y + (b.y - a.y) * t;
        p.z = a.z + (b.z - a.z) * t;
        return p;
    };

    // 走廊体积（3D box sequence）
    visualization_msgs::Marker volume;
    volume.header.frame_id = world_frame_;
    volume.header.stamp = ros::Time::now();
    volume.ns = "corridors";
    volume.id = 1;
    volume.action = visualization_msgs::Marker::ADD;
    volume.pose.orientation.w = 1.0;
    volume.type = visualization_msgs::Marker::TRIANGLE_LIST;
    volume.scale.x = 1.0;
    volume.scale.y = 1.0;
    volume.scale.z = 1.0;
    // RViz 里 TRIANGLE_LIST 即使使用 per-vertex colors，也建议把 marker.color.a 设为 >0，
    // 否则可能被当成“全透明/白板”而看不到预期颜色。
    volume.color.r = 1.0;
    volume.color.g = 1.0;
    volume.color.b = 1.0;
    volume.color.a = 1.0;

    auto addTri = [&](const geometry_msgs::Point& p1, const std_msgs::ColorRGBA& c1,
                      const geometry_msgs::Point& p2, const std_msgs::ColorRGBA& c2,
                      const geometry_msgs::Point& p3, const std_msgs::ColorRGBA& c3) {
        volume.points.push_back(p1); volume.colors.push_back(c1);
        volume.points.push_back(p2); volume.colors.push_back(c2);
        volume.points.push_back(p3); volume.colors.push_back(c3);
    };
    auto addQuad = [&](const geometry_msgs::Point& p0, const std_msgs::ColorRGBA& c0,
                       const geometry_msgs::Point& p1, const std_msgs::ColorRGBA& c1,
                       const geometry_msgs::Point& p2, const std_msgs::ColorRGBA& c2,
                       const geometry_msgs::Point& p3, const std_msgs::ColorRGBA& c3) {
        addTri(p0, c0, p1, c1, p2, c2);
        addTri(p0, c0, p2, c2, p3, c3);
    };

    volume.points.reserve((n - 1) * 24);
    volume.colors.reserve((n - 1) * 24);

    for (size_t i = 0; i + 1 < n; ++i) {
        const float v0 = valueAt(i);
        const float v1 = valueAt(i + 1);
        const float a0 = static_cast<float>(corridor_viz_fill_alpha_min_ +
                                            (corridor_viz_fill_alpha_max_ - corridor_viz_fill_alpha_min_) * v0);
        const float a1 = static_cast<float>(corridor_viz_fill_alpha_min_ +
                                            (corridor_viz_fill_alpha_max_ - corridor_viz_fill_alpha_min_) * v1);
        const std_msgs::ColorRGBA c0 = riskFillColor(v0, a0);
        const std_msgs::ColorRGBA c1 = riskFillColor(v1, a1);

        // 4 个侧面（不画端盖，避免段间重叠 z-fighting）
        addQuad(lb[i], c0, lt[i], c0, lt[i + 1], c1, lb[i + 1], c1);          // left
        addQuad(rb[i], c0, rb[i + 1], c1, rt[i + 1], c1, rt[i], c0);          // right
        addQuad(lt[i], c0, rt[i], c0, rt[i + 1], c1, lt[i + 1], c1);          // top
        addQuad(lb[i], c0, lb[i + 1], c1, rb[i + 1], c1, rb[i], c0);          // bottom
    }
    pub_corridor_marker_.publish(volume);

    // 边界线：更深色、(可选) 虚线
    size_t outline_point_count = 0;
    size_t grid_point_count = 0;
    auto addLine = [&](visualization_msgs::Marker& m,
                       const geometry_msgs::Point& a,
                       const geometry_msgs::Point& b,
                       const std_msgs::ColorRGBA& col) {
        m.points.push_back(a); m.colors.push_back(col);
        m.points.push_back(b); m.colors.push_back(col);
    };
    auto addDashedLine = [&](visualization_msgs::Marker& m,
                             const geometry_msgs::Point& a,
                             const geometry_msgs::Point& b,
                             const std_msgs::ColorRGBA& col,
                             double dash_len) {
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double dz = b.z - a.z;
        const double L = std::hypot(std::hypot(dx, dy), dz);
        if (L < 1e-6 || dash_len <= 1e-6) {
            addLine(m, a, b, col);
            return;
        }
        const double ux = dx / L;
        const double uy = dy / L;
        const double uz = dz / L;
        const double step = 2.0 * dash_len;
        for (double s = 0.0; s < L; s += step) {
            const double e = std::min(L, s + dash_len);
            geometry_msgs::Point p0, p1;
            p0.x = a.x + ux * s; p0.y = a.y + uy * s; p0.z = a.z + uz * s;
            p1.x = a.x + ux * e; p1.y = a.y + uy * e; p1.z = a.z + uz * e;
            addLine(m, p0, p1, col);
        }
    };

    if (corridor_viz_outline_enable_) {
        visualization_msgs::Marker outline;
        outline.header.frame_id = world_frame_;
        outline.header.stamp = volume.header.stamp;
        outline.ns = "corridors";
        outline.id = 2;
        outline.action = visualization_msgs::Marker::ADD;
        outline.pose.orientation.w = 1.0;
        outline.type = visualization_msgs::Marker::LINE_LIST;
        outline.scale.x = corridor_viz_outline_width_;
        outline.color.a = 1.0;

        outline.points.reserve((n - 1) * 16 + n * 8);
        outline.colors.reserve((n - 1) * 16 + n * 8);

        for (size_t i = 0; i < n; ++i) {
            const float v = valueAt(i);
            const std_msgs::ColorRGBA col = tealLineColor(v, static_cast<float>(corridor_viz_outline_alpha_), 0.85f);

            geometry_msgs::Point lb_o = lb[i], rb_o = rb[i], lt_o = lt[i], rt_o = rt[i];
            lb_o.z = z_bottom_outline; rb_o.z = z_bottom_outline;
            lt_o.z = z_top_outline;    rt_o.z = z_top_outline;

            const auto addEdge = [&](const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
                if (corridor_viz_outline_dashed_) {
                    addDashedLine(outline, a, b, col, corridor_viz_dash_length_);
                } else {
                    addLine(outline, a, b, col);
                }
            };

            // 截面矩形
            addEdge(lb_o, rb_o);
            addEdge(rb_o, rt_o);
            addEdge(rt_o, lt_o);
            addEdge(lt_o, lb_o);

            // 纵向 4 条棱线
            if (i + 1 < n) {
                geometry_msgs::Point lb1 = lb[i + 1], rb1 = rb[i + 1], lt1 = lt[i + 1], rt1 = rt[i + 1];
                lb1.z = z_bottom_outline; rb1.z = z_bottom_outline;
                lt1.z = z_top_outline;    rt1.z = z_top_outline;

                addEdge(lb_o, lb1);
                addEdge(rb_o, rb1);
                addEdge(lt_o, lt1);
                addEdge(rt_o, rt1);
            }
        }
        outline_point_count = outline.points.size();
        pub_corridor_marker_.publish(outline);
    } else {
        publishDelete(2);
    }

    // 内部网格：切片矩形 + 纵向连接（形成可透视的格网结构）
    if (corridor_viz_grid_enable_ && corridor_viz_grid_step_m_ > 1e-3) {
        visualization_msgs::Marker grid;
        grid.header.frame_id = world_frame_;
        grid.header.stamp = volume.header.stamp;
        grid.ns = "corridors";
        grid.id = 4;
        grid.action = visualization_msgs::Marker::ADD;
        grid.pose.orientation.w = 1.0;
        grid.type = visualization_msgs::Marker::LINE_LIST;
        grid.scale.x = corridor_viz_grid_width_;
        grid.color.a = 1.0;

        for (size_t i = 0; i + 1 < n; ++i) {
            const float v0 = valueAt(i);
            const float v1 = valueAt(i + 1);
            const double dx = center[i + 1].x - center[i].x;
            const double dy = center[i + 1].y - center[i].y;
            const double L = std::hypot(dx, dy);
            const int slices = std::max(1, static_cast<int>(std::floor(L / corridor_viz_grid_step_m_)));

            geometry_msgs::Point prev_lb, prev_rb, prev_lt, prev_rt;
            for (int s = 0; s <= slices; ++s) {
                const double t = (slices <= 0) ? 0.0 : static_cast<double>(s) / static_cast<double>(slices);
                geometry_msgs::Point slb = lerpPoint(lb[i], lb[i + 1], t);
                geometry_msgs::Point srb = lerpPoint(rb[i], rb[i + 1], t);
                geometry_msgs::Point slt = lerpPoint(lt[i], lt[i + 1], t);
                geometry_msgs::Point srt = lerpPoint(rt[i], rt[i + 1], t);
                slb.z = z_bottom_grid; srb.z = z_bottom_grid;
                slt.z = z_top_grid;    srt.z = z_top_grid;

                const float vv = clampf(v0 + static_cast<float>(t) * (v1 - v0), 0.0f, 1.0f);
                const std_msgs::ColorRGBA col = tealLineColor(vv, static_cast<float>(corridor_viz_grid_alpha_), 1.0f);

                // slice rectangle
                addLine(grid, slb, srb, col);
                addLine(grid, srb, srt, col);
                addLine(grid, srt, slt, col);
                addLine(grid, slt, slb, col);

                // cross inside (gives a "mesh" feel through transparency)
                geometry_msgs::Point mid_b, mid_t, mid_l, mid_r;
                mid_b = lerpPoint(slb, srb, 0.5);
                mid_t = lerpPoint(slt, srt, 0.5);
                mid_l = lerpPoint(slb, slt, 0.5);
                mid_r = lerpPoint(srb, srt, 0.5);
                addLine(grid, mid_b, mid_t, col);
                addLine(grid, mid_l, mid_r, col);

                if (s > 0) {
                    addLine(grid, prev_lb, slb, col);
                    addLine(grid, prev_rb, srb, col);
                    addLine(grid, prev_lt, slt, col);
                    addLine(grid, prev_rt, srt, col);
                }
                prev_lb = slb; prev_rb = srb; prev_lt = slt; prev_rt = srt;
            }
        }
        grid_point_count = grid.points.size();
        pub_corridor_marker_.publish(grid);
    } else {
        publishDelete(4);
    }

    // 可选：中心线（辅助观察轨迹方向）
    if (corridor_viz_centerline_enable_) {
        visualization_msgs::Marker centerline;
        centerline.header.frame_id = world_frame_;
        centerline.header.stamp = volume.header.stamp;
        centerline.ns = "corridors";
        centerline.id = 3;
        centerline.action = visualization_msgs::Marker::ADD;
        centerline.pose.orientation.w = 1.0;
        centerline.type = visualization_msgs::Marker::LINE_STRIP;
        centerline.scale.x = corridor_viz_centerline_width_;
        centerline.color.r = 0.95;
        centerline.color.g = 0.95;
        centerline.color.b = 0.95;
        centerline.color.a = 0.9;

        centerline.points.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            geometry_msgs::Point p = center[i];
            p.z = z_centerline;
            centerline.points.push_back(p);
        }
        pub_corridor_marker_.publish(centerline);
    } else {
        publishDelete(3);
    }

    ROS_INFO_THROTTLE(2.0, "corridor viz: n=%zu tris=%zu outline_lines=%zu grid_lines=%zu (gen took %.1f ms)",
                      n,
                      volume.points.size() / 3,
                      outline_point_count / 2,
                      grid_point_count / 2,
                      msSince(t_start));
}

void PathPlanner::publishPath(const std::vector<Eigen::Vector2i>& path_indices, const Eigen::Vector2i& goal_idx) {
    const auto t_publish_path_start = std::chrono::steady_clock::now();
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = world_frame_;

    for (const auto& idx : path_indices) {
        geometry_msgs::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position = gridToWorld(idx);
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }
    
    if(!path_indices.empty()){
        path_msg.poses.back().pose.position = gridToWorld(goal_idx);
    }

    // 给 /plan 的每个点写入切线朝向（yaw），供走廊安全检查/跟踪器使用
    if (path_msg.poses.size() >= 2) {
        for (size_t i = 0; i < path_msg.poses.size(); ++i) {
            const auto& p = path_msg.poses[i].pose.position;
            geometry_msgs::Point p_prev = p;
            geometry_msgs::Point p_next = p;
            if (i > 0) p_prev = path_msg.poses[i - 1].pose.position;
            if (i + 1 < path_msg.poses.size()) p_next = path_msg.poses[i + 1].pose.position;
            const double yaw = std::atan2(p_next.y - p_prev.y, p_next.x - p_prev.x);
            path_msg.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        }
    }

    pub_path_.publish(path_msg);
    publishPathMarker(path_msg);
    last_published_path_ = path_msg; // 保存路径
	last_planned_map_version_ = map_version_;  // 记录本次规划使用的地图版本
    // 从 nav_msgs::Path 生成并发布 GlobalPath（GRP）
    ROS_INFO("About to publish GRP: path_points=%zu, grid_info_=%s",
             path_msg.poses.size(), grid_info_ ? "valid" : "null");
    const auto t_grp = std::chrono::steady_clock::now();
    publishGRP(path_msg);
    ROS_INFO_THROTTLE(1.0, "Timing(ms): publishGRP=%.1f (publishPath total=%.1f)",
                      msSince(t_grp), msSince(t_publish_path_start));
    ROS_INFO("GRP publish completed");

}

void PathPlanner::publishGRP(const nav_msgs::Path& path_msg) {
    const auto t_grp_total = std::chrono::steady_clock::now();
    if (path_msg.poses.empty() || !grid_info_) return;

    // ★ 关键修复：检查地图版本一致性
    if (last_planned_map_version_ != map_version_) {
        // 路径是旧地图上规划的，但现在是新地图
        // 有两种选择：
        // 1) 不发布 GRP，等待下次规划更新
        // 2) 仍然发布，但使用保守的置信度（灰色）表示"数据不一致"

        ROS_WARN_THROTTLE(2.0,
            "Path version mismatch: path planned on map %d, current map %d. "
            "Using conservative confidence visualization.",
            last_planned_map_version_, map_version_);

        // 选择方案2：使用保守置信度发布，避免误导
        // 1) path -> 栅格索引序列
        const auto t_to_idx = std::chrono::steady_clock::now();
        std::vector<Eigen::Vector2i> path_idx = pathMsgToIndices(path_msg);
        const double ms_to_idx = msSince(t_to_idx);

        // 2) 智能重采样
        const auto t_resample = std::chrono::steady_clock::now();
        if (safe_resample_enable_ && grp_resample_step_m_ > 1e-3) {
            size_t original_size = path_idx.size();
            path_idx = intelligentSafeResample(path_idx, grp_resample_step_m_);
            ROS_INFO("Old map - Intelligent resampling: %lu -> %lu waypoints", original_size, path_idx.size());
        } else if (grp_resample_enable_ && grp_resample_step_m_ > 1e-3) {
            // 保持原有逻辑作为回退
            path_idx = resamplePath(path_idx, grp_resample_step_m_);
        }
        const double ms_resample = msSince(t_resample);

        // 3) 地图版本不一致：GRP 用“不可用”数据（宽度/置信度均为 0），避免下游误用；
        //    可视化仍可单独给出保守显示（见下方 half_width_viz / confidence_viz）。
        std::vector<float> half_width_grp(path_idx.size(), 0.0f);
        std::vector<float> confidence_grp(path_idx.size(), 0.0f);

        std::vector<float> half_width_viz(path_idx.size(), static_cast<float>(corridor_min_width_ * 0.5));
        std::vector<float> confidence_viz(path_idx.size(), 0.0f);

        // 4) 组装消息，标记为旧地图版本
        const auto t_msg = std::chrono::steady_clock::now();
        fitplane_planner::GlobalPath grp;
        grp.header.stamp    = ros::Time::now();
        grp.header.frame_id = world_frame_;
        grp.map_version     = (last_planned_map_version_ < 0) ? 0u : static_cast<uint32_t>(last_planned_map_version_);

        grp.poses.reserve(path_idx.size());
        grp.corridor_half_width.reserve(path_idx.size());
        grp.path_confidence.reserve(path_idx.size());

        for (size_t i = 0; i < path_idx.size(); ++i) {
            geometry_msgs::Pose p;
            p.position = gridToWorld(path_idx[i]);
            const geometry_msgs::Point p_prev = (i > 0) ? gridToWorld(path_idx[i - 1]) : p.position;
            const geometry_msgs::Point p_next = (i + 1 < path_idx.size()) ? gridToWorld(path_idx[i + 1]) : p.position;
            const double yaw = std::atan2(p_next.y - p_prev.y, p_next.x - p_prev.x);
            p.orientation = tf::createQuaternionMsgFromYaw(yaw);
            grp.poses.push_back(p);
            grp.corridor_half_width.push_back(half_width_grp[i]);
            grp.path_confidence.push_back(confidence_grp[i]);
        }

        const double ms_msg = msSince(t_msg);
        const auto t_pub = std::chrono::steady_clock::now();
        ROS_INFO("Publishing GRP message (old map, path_map_version=%d, current_map_version=%u): %zu path points",
                 last_planned_map_version_, map_version_, grp.poses.size());
        pub_grp_.publish(grp);
        const double ms_pub = msSince(t_pub);

        // --- 发布保守的可视化 ---
        const auto t_viz = std::chrono::steady_clock::now();
        publishPathVisualization(path_idx, half_width_viz);
        publishConfidenceVisualization(path_idx, confidence_viz, half_width_viz);
        publishGoalMarker();
        const double ms_viz = msSince(t_viz);

        ROS_INFO_THROTTLE(1.0,
                          "Timing(ms): grp_old total=%.1f to_idx=%.1f resample=%.1f msg=%.1f pub=%.1f viz=%.1f",
                          msSince(t_grp_total), ms_to_idx, ms_resample, ms_msg, ms_pub, ms_viz);

        return; // 提前返回，不执行新地图的置信度计算
    }

    // 正常情况：路径和当前地图版本一致
    // 1) path -> 栅格索引序列
   ROS_INFO("Normal case: path_map_version=%d, current_map_version=%u",
            last_planned_map_version_, map_version_);
    const auto t_to_idx = std::chrono::steady_clock::now();
    std::vector<Eigen::Vector2i> path_idx = pathMsgToIndices(path_msg);
    const double ms_to_idx = msSince(t_to_idx);

    // 2) 智能重采样（控制点更均匀且安全）
    // TEMP: 暂时关闭重采样
    const auto t_resample = std::chrono::steady_clock::now();
    // if (safe_resample_enable_ && grp_resample_step_m_ > 1e-3) {
    //     size_t original_size = path_idx.size();
    //     path_idx = intelligentSafeResample(path_idx, grp_resample_step_m_);
    //     ROS_INFO("Intelligent resampling: %lu -> %lu waypoints (step: %.2fm)", original_size, path_idx.size(), grp_resample_step_m_);
    // } else if (grp_resample_enable_ && grp_resample_step_m_ > 1e-3) {
    //     // 保持原有逻辑作为回退
    //     path_idx = resamplePath(path_idx, grp_resample_step_m_);
    // }
    const double ms_resample = msSince(t_resample);

    // 3) 计算走廊半宽与置信度（基于当前地图）
    std::vector<float> half_width, confidence;
    const auto t_conf = std::chrono::steady_clock::now();
    computeCorridorAndConfidence(path_idx, half_width, confidence);
    const double ms_conf = msSince(t_conf);
    // 让时间滤波真正生效：更新历史置信度
    last_confidence_ = confidence;
    confidence_history_initialized_ = true;

    // 4) 组装消息
    const auto t_msg = std::chrono::steady_clock::now();
    fitplane_planner::GlobalPath grp;
    grp.header.stamp    = ros::Time::now();
    grp.header.frame_id = world_frame_;
    grp.map_version     = map_version_;

    grp.poses.reserve(path_idx.size());
    grp.corridor_half_width.reserve(path_idx.size());
    grp.path_confidence.reserve(path_idx.size());

    // 不改 msg 的前提下表达“左右非对称走廊”：
    // - 估计每个点左右单侧可用宽度 wl/wr（沿路径法向采样）
    // - 将中心线按 (wl-wr)/2 横向平移，使得 corridor_half_width=(wl+wr)/2 的对称走廊边界恰好落在 wl/wr 上
    // 这样下游仍只需要 corridor_half_width，但能获得“更不保守”的有效走廊中心线与宽度。
    const double hw_max = 0.5 * corridor_max_width_;
    const double step_m = std::max(0.01, corridor_viz_clearance_step_m_);
    const double high_cost_threshold = corridor_viz_obstacle_cost_threshold_;

    auto isBlockedCost = [&](double cost) {
        if (cost < 0.0) return true;
        if (corridor_viz_unknown_is_obstacle_ && std::abs(cost - unknown_space_cost_) < 1e-6) return true;
        if (std::isfinite(high_cost_threshold) && high_cost_threshold > 0.0 && cost >= high_cost_threshold) return true;
        return false;
    };

    auto sideClearance = [&](const Eigen::Vector2i& g, double theta, int sign) -> double {
        double last_ok = 0.0;
        for (double d = step_m; d <= hw_max + 1e-9; d += step_m) {
            const double c = getCostAtOffsetPrecise(g.x(), g.y(), theta, sign * d);
            if (isBlockedCost(c)) break;
            last_ok = d;
        }
        return std::max(0.0, last_ok - corridor_margin_);
    };

    for (size_t i = 0; i < path_idx.size(); ++i) {
        const geometry_msgs::Point base = gridToWorld(path_idx[i]);
        const geometry_msgs::Point prev = (i > 0) ? gridToWorld(path_idx[i - 1]) : base;
        const geometry_msgs::Point next = (i + 1 < path_idx.size()) ? gridToWorld(path_idx[i + 1]) : base;
        double yaw = std::atan2(next.y - prev.y, next.x - prev.x);
        if (std::hypot(next.x - prev.x, next.y - prev.y) < 1e-6) yaw = 0.0;

        geometry_msgs::Pose p;
        p.position = base;
        p.orientation = tf::createQuaternionMsgFromYaw(yaw);

        float hw_out = half_width[i];
        if (grp_asym_enable_) {
            // 左/右单侧清空度（米）
            double wl = sideClearance(path_idx[i], yaw, +1);
            double wr = sideClearance(path_idx[i], yaw, -1);
            wl = std::max(0.0, std::min(wl, hw_max));
            wr = std::max(0.0, std::min(wr, hw_max));

            // 保持与 computeCorridorAndConfidence 一致：unknown 多时 shrink（保守）
            const double unknown_ratio = computeUnknownRatio(path_idx[i], conf_local_radius_cells_);
            const double normalized_unknown = clampf(static_cast<float>(std::min(unknown_ratio, 0.7)), 0.0f, 1.0f);
            const double s_unknown = 1.0 - std::pow(normalized_unknown, 2.0);
            const double shrink_factor = 0.1 + 0.9 * s_unknown;
            wl *= shrink_factor;
            wr *= shrink_factor;

            const double half = std::max(0.0, std::min(0.5 * (wl + wr), hw_max));
            const double shift = std::max(-half, std::min(half, 0.5 * (wl - wr)));

            // 左法向（与 getCostAtOffsetPrecise 一致）
            const double nx = -std::sin(yaw);
            const double ny =  std::cos(yaw);
            p.position.x += nx * shift;
            p.position.y += ny * shift;
            hw_out = static_cast<float>(half);
        }

        grp.poses.push_back(p);
        grp.corridor_half_width.push_back(hw_out);
        grp.path_confidence.push_back(confidence[i]);
    }

    const double ms_msg = msSince(t_msg);
    const auto t_pub = std::chrono::steady_clock::now();
    ROS_INFO("Publishing GRP message (map_version=%u): %zu path points", map_version_, grp.poses.size());
    pub_grp_.publish(grp);
    publishGrpCenterlineMarker(grp.poses);
    const double ms_pub = msSince(t_pub);

    // --- 发布路径与走廊可视化 ---
    const auto t_viz = std::chrono::steady_clock::now();
    publishPathVisualization(path_idx, half_width);  // 路径的可视化
    publishConfidenceVisualization(path_idx, confidence, half_width);  // 走廊可视化
    publishGoalMarker();  // 发布目标点的标记
    const double ms_viz = msSince(t_viz);

    ROS_INFO_THROTTLE(1.0,
                      "Timing(ms): grp total=%.1f to_idx=%.1f resample=%.1f conf=%.1f msg=%.1f pub=%.1f viz=%.1f",
                      msSince(t_grp_total), ms_to_idx, ms_resample, ms_conf, ms_msg, ms_pub, ms_viz);
}

// >>> 新实现：探索模式版安全走廊 & 置信度计算
void PathPlanner::computeCorridorAndConfidence(
    const std::vector<Eigen::Vector2i>& path_indices,
    std::vector<float>& half_width,
    std::vector<float>& confidence) const
{
    half_width.resize(path_indices.size(), 0.0f);
    confidence.resize(path_indices.size(), 0.0f);

    if (!grid_info_ || path_indices.empty()) return;

    const int    W   = grid_info_->info.width;
    const int    H   = grid_info_->info.height;
    const double res = grid_info_->info.resolution;

    // 只限制最大半宽（单位：m），不再用 corridor_min_width_ “硬撑”几何边界
    const double hw_max = 0.5 * corridor_max_width_;

    // 归一化用总权重
    const double sum_w = conf_w_trav_ + conf_w_slope_ + conf_w_unknown_ + conf_w_clearance_;
    const double eps   = 1e-6;

    for (size_t i = 0; i < path_indices.size(); ++i) {
        const auto& g = path_indices[i];

        // 越界：给一个最保守的结果
        if (g.x() < 0 || g.x() >= W || g.y() < 0 || g.y() >= H) {
            half_width[i] = 0.0f;
            confidence[i] = 0.0f;
            continue;
        }

        const int idx = g.y() * W + g.x();

        // ------------------------------
        // 1) 几何清空度 -> 走廊半宽上限
        // ------------------------------
        // 改进：走廊半宽用“沿路径法向的单侧清空度”估计，而不是用距离场的各向同性距离。
        // 否则：障碍物在前/后方也会导致 half_width 被不必要压缩。
        const double step_m = 0.10;  // 采样步长（m）
        const double high_cost_threshold = 80.0;  // 与距离场 seed 一致：膨胀高代价区也视为不可进入

        const geometry_msgs::Point p_prev = (i > 0) ? gridToWorld(path_indices[i - 1]) : gridToWorld(g);
        const geometry_msgs::Point p_next = (i + 1 < path_indices.size()) ? gridToWorld(path_indices[i + 1]) : gridToWorld(g);
        double theta = std::atan2(p_next.y - p_prev.y, p_next.x - p_prev.x);
        if (std::hypot(p_next.x - p_prev.x, p_next.y - p_prev.y) < 1e-6) {
            theta = 0.0;
        }

        auto sideClearance = [&](int sign) -> double {
            double last_ok = 0.0;
            for (double d = step_m; d <= hw_max + 1e-9; d += step_m) {
                const double c = getCostAtOffsetPrecise(g.x(), g.y(), theta, sign * d);
                if (c < 0.0 || c > high_cost_threshold) break;
                last_ok = d;
            }
            return std::max(0.0, last_ok - corridor_margin_);
        };

        double left_clear = sideClearance(+1);
        double right_clear = sideClearance(-1);
        double hw_geom = std::min(left_clear, right_clear);

        // 未知区域：保持保守（避免 unknown 的“无限扩张”）
        if (idx >= 0 && idx < static_cast<int>(stable_cells_.size()) && stable_cells_[idx].state == -1) {
            const double conservative_max_width = std::min(corridor_max_width_, 8.0);
            hw_geom = std::min(hw_geom, 0.5 * conservative_max_width);
        }

        const double clearance_m = hw_geom;
        double hw = std::min(hw_geom, hw_max);

        // ------------------------------
        // 2) 置信度各分量（0..1）
        // ------------------------------

        // 2.1 traversability（越小越好） - nav_msgs::OccupancyGrid doesn't have this field
        // Use default traversability of 0.5 (neutral value)
        double trav = 0.5;
        double s_trav = clampf(static_cast<float>(1.0 - trav), 0.0f, 1.0f);

        // 2.2 slope（越小越好） - nav_msgs::OccupancyGrid doesn't have this field
        // Use default slope of 0 degrees (flat)
        double slope = 0.0;
        double s_slope = 1.0;

        // 2.3 unknown（邻域未知率越低越好）- 钝化处理，避免因 stable_cells_ 记忆延迟导致的敏感变化
        double unknown_ratio = computeUnknownRatio(g, conf_local_radius_cells_);

        // ★ 钝化处理1：设置 unknown_ratio 上限，避免极端情况
        unknown_ratio = std::min(unknown_ratio, 0.7);  // 最多70%未知，避免把置信度"炸穿地心"

        // ★ 钝化处理2：使用凸函数，让少量未知对置信度影响更小
        // 只有未知比例很高时才会明显抑制置信度
        double normalized_unknown = clampf(static_cast<float>(unknown_ratio), 0.0f, 1.0f);
        double s_unknown = 1.0 - std::pow(normalized_unknown, 2.0);  // 凸函数：(0,1) -> (1,0)

        // 举例：
        // unknown_ratio=0.1 -> s_unknown≈0.99 (几乎不影响)
        // unknown_ratio=0.3 -> s_unknown≈0.91 (轻微影响)
        // unknown_ratio=0.6 -> s_unknown≈0.64 (明显影响)
        // unknown_ratio=0.7 -> s_unknown≈0.51 (很强影响)

        // 2.4 clearance（越大越好）- 平滑处理，避免硬阈值导致的闪烁
        double s_clear = 0.0f;
        if (clearance_m <= 0.0) {
            // 贴障碍或超出边界：clearance 分量为 0，但不直接一票否决
            s_clear = 0.0f;
        } else {
            // 正常情况：基于 clearance_ref_m_ 进行归一化
            s_clear = clampf(
                static_cast<float>(clearance_m / std::max(0.1, clearance_ref_m_)),
                0.0f, 1.0f);
        }

        // ------------------------------
        // 3) 探索模式下的半宽缩放：
        //    - unknown 越多 -> 走廊越窄
        //    - 保持能穿 unknown，但可视化上表示"风险更大"
        // ------------------------------
        // 强化缩放：shrink_factor ∈ [0.1, 1.0]，unknown_ratio=1 时变成 0.1，unknown_ratio=0 时为 1.0
        // 这样未知区域的走廊会明显变窄，更能反映不确定性
        double shrink_factor = 0.1 + 0.9 * s_unknown;

        // 可选：如果希望指数型缩放（更激进），可以启用下面这行：
        // double shrink_factor = std::exp(-2.0 * (1.0 - s_unknown));  // unknown多时快速缩小

        hw *= shrink_factor;

        // 定位到 [0, hw_max]，防止数值问题
        hw = clampf(static_cast<float>(hw), 0.0f, static_cast<float>(hw_max));
        half_width[i] = static_cast<float>(hw);

        // ★ 移除硬阈值：不再因为 clearance_m <= 0 就直接置零置信度
        // 让总置信度由四个分量平滑加权决定，避免边界抖动导致的红块闪烁

        // ------------------------------
        // 4) 加权融合得到总置信度 c_i
        // ------------------------------
        double c = (conf_w_trav_      * s_trav
                 +  conf_w_slope_     * s_slope
                 +  conf_w_unknown_   * s_unknown
                 +  conf_w_clearance_ * s_clear) / std::max(sum_w, eps);

        float new_confidence = clampf(static_cast<float>(c), 0.0f, 1.0f);

        // ★ 时间滤波：平滑置信度变化，减少因 stable_cells_ 记忆延迟导致的闪烁
        if (confidence_history_initialized_ && i < last_confidence_.size()) {
            // 使用指数移动平均：0.7*old + 0.3*new
            confidence[i] = confidence_alpha_ * last_confidence_[i] +
                           (1.0f - confidence_alpha_) * new_confidence;
        } else {
            // 首次运行或历史长度不匹配，直接使用新值
            confidence[i] = new_confidence;
        }
    }
}

// <<< 新实现结束

// >>> 新增：Path -> 栅格索引序列
std::vector<Eigen::Vector2i> PathPlanner::pathMsgToIndices(const nav_msgs::Path& path) const
{
    std::vector<Eigen::Vector2i> out;
    out.reserve(path.poses.size());
    for (const auto& ps : path.poses) {
        out.push_back(worldToGrid(ps.pose.position));
    }
    return out;
}

void PathPlanner::publishPathMarker(const nav_msgs::Path& path) {
    if (path.poses.empty() && pub_path_marker_.getNumSubscribers() > 0) {
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = path.header.frame_id;
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "path";
        line_strip.action = visualization_msgs::Marker::DELETE;
        line_strip.pose.orientation.w = 1.0;
        line_strip.id = 0;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        
        pub_path_marker_.publish(line_strip);
    } else if (!path.poses.empty()) {
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = path.header.frame_id;
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "path";
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;
        line_strip.id = 0;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        
        // 设置线条宽度和颜色
        line_strip.scale.x = path_marker_width_;
        line_strip.color.g = 1.0; // 绿色
        line_strip.color.a = 1.0;

        for (const auto& pose_stamped : path.poses) {
            line_strip.points.push_back(pose_stamped.pose.position);
        }

        pub_path_marker_.publish(line_strip);
    }
}



void PathPlanner::publishGoalMarker(bool delete_marker) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = world_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "goal_point";
    marker.id = 0;
    
    if (delete_marker) {
        marker.action = visualization_msgs::Marker::DELETE;
    } else {
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.pose = current_goal_.pose;
        marker.scale.x = 0.4;
        marker.scale.y = 0.4;
        marker.scale.z = 0.4;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.8;
    }

    pub_goal_marker_.publish(marker);
}

} // namespace fitplane_planner
