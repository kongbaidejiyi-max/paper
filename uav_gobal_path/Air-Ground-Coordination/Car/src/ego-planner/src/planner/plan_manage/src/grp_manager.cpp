#include <plan_manage/grp_manager.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace ego_planner
{

  void GRPManager::setParam(ros::NodeHandle &nh)
  {
    nh.param("grp/default_half_width", default_half_width_, 1.0);
    nh.param("grp/min_point_spacing", min_point_spacing_, 0.2);
    nh.param("grp/max_points", max_points_, 2000);

    nh.param("grp/lambda_min", lambda_min_, 0.0);
    nh.param("grp/lambda_max", lambda_max_, 0.5);
    nh.param("grp/eval_step", eval_step_, 0.2);
  }

  bool GRPManager::update(const fitplane_planner::GlobalPath &msg)
  {
    stamp_ = msg.header.stamp;
    map_version_ = msg.map_version;

    pts_.clear();
    pts_.reserve(msg.poses.size());
    for (const auto &pose : msg.poses)
    {
      Eigen::Vector3d p;
      p << pose.position.x, pose.position.y, 0.0;
      pts_.push_back(p);
    }

    half_width_.assign(msg.corridor_half_width.begin(), msg.corridor_half_width.end());
    confidence_.assign(msg.path_confidence.begin(), msg.path_confidence.end());

    downsamplePath();
    rebuildArcLength();

    valid_ = (pts_.size() >= 2 && total_len_ > 1e-3);
    return valid_;
  }

  void GRPManager::downsamplePath()
  {
    if (pts_.size() <= 2)
      return;

    std::vector<Eigen::Vector3d> kept_pts;
    std::vector<float> kept_w;
    std::vector<float> kept_c;

    kept_pts.reserve(std::min<size_t>(pts_.size(), static_cast<size_t>(max_points_)));
    kept_w.reserve(kept_pts.capacity());
    kept_c.reserve(kept_pts.capacity());

    auto widthAtIdx = [&](size_t idx) -> float
    {
      if (!half_width_.empty() && idx < half_width_.size())
        return half_width_[idx];
      return static_cast<float>(default_half_width_);
    };
    auto confAtIdx = [&](size_t idx) -> float
    {
      if (!confidence_.empty() && idx < confidence_.size())
        return std::min(1.0f, std::max(0.0f, confidence_[idx]));
      return 1.0f;
    };

    kept_pts.push_back(pts_.front());
    kept_w.push_back(widthAtIdx(0));
    kept_c.push_back(confAtIdx(0));

    Eigen::Vector3d last_kept = pts_.front();
    for (size_t i = 1; i + 1 < pts_.size(); ++i)
    {
      if ((pts_[i] - last_kept).norm() < min_point_spacing_)
        continue;
      kept_pts.push_back(pts_[i]);
      kept_w.push_back(widthAtIdx(i));
      kept_c.push_back(confAtIdx(i));
      last_kept = pts_[i];
      if ((int)kept_pts.size() >= max_points_)
        break;
    }

    if (kept_pts.back() != pts_.back())
    {
      kept_pts.push_back(pts_.back());
      kept_w.push_back(widthAtIdx(pts_.size() - 1));
      kept_c.push_back(confAtIdx(pts_.size() - 1));
    }

    pts_.swap(kept_pts);
    half_width_.swap(kept_w);
    confidence_.swap(kept_c);
  }

  void GRPManager::rebuildArcLength()
  {
    acc_len_.clear();
    acc_len_.resize(pts_.size(), 0.0);

    total_len_ = 0.0;
    for (size_t i = 1; i < pts_.size(); ++i)
    {
      total_len_ += (pts_[i] - pts_[i - 1]).norm();
      acc_len_[i] = total_len_;
    }
  }

  bool GRPManager::findNearest(const Eigen::Vector3d &p, NearestResult &out) const
  {
    out = NearestResult{};
    if (pts_.size() < 2)
      return false;

    double best_d2 = std::numeric_limits<double>::infinity();
    int best_i = 0;
    double best_t = 0.0;

    for (int i = 0; i < (int)pts_.size() - 1; ++i)
    {
      const Eigen::Vector3d a = pts_[i];
      const Eigen::Vector3d b = pts_[i + 1];
      const Eigen::Vector3d d = b - a;
      const double d2 = d.squaredNorm();
      double t = 0.0;
      if (d2 > 1e-12)
        t = (p - a).dot(d) / d2;
      t = std::min(1.0, std::max(0.0, t));
      const Eigen::Vector3d proj = a + t * d;
      const double dist2 = (p - proj).squaredNorm();
      if (dist2 < best_d2)
      {
        best_d2 = dist2;
        best_i = i;
        best_t = t;
      }
    }

    out.ok = true;
    out.seg_idx = best_i;
    out.seg_t = best_t;
    const double seg_len = (pts_[best_i + 1] - pts_[best_i]).norm();
    out.arc_s = acc_len_[best_i] + best_t * seg_len;
    return true;
  }

  bool GRPManager::sampleByArc(double s, Eigen::Vector3d &p, Eigen::Vector3d &tangent) const
  {
    if (pts_.size() < 2)
      return false;

    if (s <= 0.0)
    {
      p = pts_.front();
      tangent = (pts_[1] - pts_[0]);
      if (tangent.norm() > 1e-6)
        tangent.normalize();
      return true;
    }
    if (s >= total_len_)
    {
      p = pts_.back();
      tangent = (pts_.back() - pts_[pts_.size() - 2]);
      if (tangent.norm() > 1e-6)
        tangent.normalize();
      return true;
    }

    auto it = std::upper_bound(acc_len_.begin(), acc_len_.end(), s);
    int idx = std::max(0, (int)std::distance(acc_len_.begin(), it) - 1);
    idx = std::min(idx, (int)pts_.size() - 2);

    const double s0 = acc_len_[idx];
    const double s1 = acc_len_[idx + 1];
    const double seg_len = std::max(1e-9, s1 - s0);
    const double t = (s - s0) / seg_len;

    const Eigen::Vector3d a = pts_[idx];
    const Eigen::Vector3d b = pts_[idx + 1];
    p = a + t * (b - a);

    tangent = (b - a);
    if (tangent.norm() > 1e-6)
      tangent.normalize();
    return true;
  }

  float GRPManager::halfWidthAt(int seg_idx, double seg_t) const
  {
    if (half_width_.empty())
      return static_cast<float>(default_half_width_);
    const int i0 = std::min(std::max(seg_idx, 0), (int)half_width_.size() - 1);
    const int i1 = std::min(i0 + 1, (int)half_width_.size() - 1);
    const float w0 = std::max(0.0f, half_width_[i0]);
    const float w1 = std::max(0.0f, half_width_[i1]);
    return w0 + (float)seg_t * (w1 - w0);
  }

  float GRPManager::confidenceAt(int seg_idx, double seg_t) const
  {
    if (confidence_.empty())
      return 1.0f;
    const int i0 = std::min(std::max(seg_idx, 0), (int)confidence_.size() - 1);
    const int i1 = std::min(i0 + 1, (int)confidence_.size() - 1);
    const float c0 = std::min(1.0f, std::max(0.0f, confidence_[i0]));
    const float c1 = std::min(1.0f, std::max(0.0f, confidence_[i1]));
    return c0 + (float)seg_t * (c1 - c0);
  }

  bool GRPManager::getLocalTarget(const Eigen::Vector3d &current_pos, double horizon, double max_vel, double max_acc,
                                  Eigen::Vector3d &local_target_pt, Eigen::Vector3d &local_target_vel) const
  {
    if (!valid_)
      return false;

    NearestResult nearest;
    if (!findNearest(current_pos, nearest))
      return false;

    const double s_target = std::min(total_len_, nearest.arc_s + std::max(0.0, horizon));
    Eigen::Vector3d tangent;
    if (!sampleByArc(s_target, local_target_pt, tangent))
      return false;

    const double remaining = total_len_ - s_target;
    const double stop_dist = (max_vel * max_vel) / (2.0 * std::max(1e-3, max_acc));
    if (remaining < stop_dist)
      local_target_vel = Eigen::Vector3d::Zero();
    else
      local_target_vel = tangent * max_vel;

    return true;
  }

  bool GRPManager::exportSubPath(const Eigen::Vector3d &current_pos, double forward_length,
                                 std::vector<Eigen::Vector3d> &pts, std::vector<float> &half_width,
                                 std::vector<float> &confidence) const
  {
    pts.clear();
    half_width.clear();
    confidence.clear();
    if (!valid_)
      return false;

    NearestResult nearest;
    if (!findNearest(current_pos, nearest))
      return false;

    const double s0 = nearest.arc_s;
    const double s1 = std::min(total_len_, s0 + std::max(0.0, forward_length));
    if (s1 - s0 < 1e-3)
      return false;

    Eigen::Vector3d p, t;
    for (double s = s0; s <= s1 + 1e-6; s += std::max(1e-3, eval_step_))
    {
      if (!sampleByArc(s, p, t))
        break;

      NearestResult near_s;
      if (!findNearest(p, near_s))
        continue;

      pts.push_back(p);
      half_width.push_back(halfWidthAt(near_s.seg_idx, near_s.seg_t));
      confidence.push_back(confidenceAt(near_s.seg_idx, near_s.seg_t));
    }

    if (pts.size() < 2)
      return false;
    return true;
  }

  double GRPManager::computeAdaptiveLambda(const GridMap::Ptr &grid_map, const Eigen::Vector3d &current_pos, double horizon,
                                           double *out_conf_mean, double *out_unknown_rate, double *out_conflict_rate) const
  {
    if (!valid_ || grid_map == nullptr)
      return 0.0;

    std::vector<Eigen::Vector3d> pts;
    std::vector<float> widths;
    std::vector<float> confs;
    if (!exportSubPath(current_pos, std::max(0.0, horizon), pts, widths, confs))
      return 0.0;

    double conf_sum = 0.0;
    int unknown_cnt = 0;
    int conflict_cnt = 0;
    int cnt = 0;
    for (size_t i = 0; i < pts.size(); ++i)
    {
      const Eigen::Vector3d &p = pts[i];
      const float c = (i < confs.size()) ? std::min(1.0f, std::max(0.0f, confs[i])) : 1.0f;

      conf_sum += c;
      cnt++;

      if (!grid_map->isInMap(p) || grid_map->isUnknown(p))
        unknown_cnt++;
      if (grid_map->getInflateOccupancy(p) == 1)
        conflict_cnt++;
    }

    if (cnt <= 0)
      return 0.0;

    const double conf_mean = conf_sum / (double)cnt;
    const double unknown_rate = (double)unknown_cnt / (double)cnt;
    const double conflict_rate = (double)conflict_cnt / (double)cnt;

    if (out_conf_mean)
      *out_conf_mean = conf_mean;
    if (out_unknown_rate)
      *out_unknown_rate = unknown_rate;
    if (out_conflict_rate)
      *out_conflict_rate = conflict_rate;

    const double lambda_raw = lambda_max_ * conf_mean * (1.0 - unknown_rate) * (1.0 - conflict_rate);
    const double lambda = std::min(lambda_max_, std::max(lambda_min_, lambda_raw));
    return lambda;
  }

} // namespace ego_planner
