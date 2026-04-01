#ifndef _GRP_MANAGER_H_
#define _GRP_MANAGER_H_

#include <Eigen/Eigen>
#include <plan_env/grid_map.h>
#include <ros/ros.h>

#include <cstdint>
#include <string>
#include <vector>

#include <fitplane_planner/GlobalPath.h>

namespace ego_planner
{

  class GRPManager
  {
  public:
    GRPManager() = default;
    ~GRPManager() = default;

    void setParam(ros::NodeHandle &nh);
    bool update(const fitplane_planner::GlobalPath &msg);

    bool valid() const { return valid_; }
    uint32_t mapVersion() const { return map_version_; }
    ros::Time stamp() const { return stamp_; }

    bool getLocalTarget(const Eigen::Vector3d &current_pos, double horizon, double max_vel, double max_acc,
                        Eigen::Vector3d &local_target_pt, Eigen::Vector3d &local_target_vel) const;

    bool exportSubPath(const Eigen::Vector3d &current_pos, double forward_length,
                       std::vector<Eigen::Vector3d> &pts, std::vector<float> &half_width,
                       std::vector<float> &confidence) const;

    double computeAdaptiveLambda(const GridMap::Ptr &grid_map, const Eigen::Vector3d &current_pos, double horizon,
                                 double *out_conf_mean = nullptr, double *out_unknown_rate = nullptr,
                                 double *out_conflict_rate = nullptr) const;

  private:
    struct NearestResult
    {
      bool ok{false};
      int seg_idx{0};
      double seg_t{0.0};
      double arc_s{0.0};
    };

    bool findNearest(const Eigen::Vector3d &p, NearestResult &out) const;
    bool sampleByArc(double s, Eigen::Vector3d &p, Eigen::Vector3d &tangent) const;
    void rebuildArcLength();
    void downsamplePath();

    float halfWidthAt(int seg_idx, double seg_t) const;
    float confidenceAt(int seg_idx, double seg_t) const;

  private:
    bool valid_{false};
    uint32_t map_version_{0};
    ros::Time stamp_{0.0};

    std::vector<Eigen::Vector3d> pts_;
    std::vector<float> half_width_;
    std::vector<float> confidence_;
    std::vector<double> acc_len_;
    double total_len_{0.0};

    double default_half_width_{1.0};
    double min_point_spacing_{0.2};
    int max_points_{2000};

    double lambda_min_{0.0};
    double lambda_max_{0.5};
    double eval_step_{0.2};
  };

} // namespace ego_planner

#endif

