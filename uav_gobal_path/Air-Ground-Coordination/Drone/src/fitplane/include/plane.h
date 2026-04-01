
#ifndef PLANE_H
#define PLANE_H

#include "World.h"
#include <fitplane/PlaneMap.h>
#include <fitplane/Plane.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/filters/voxel_grid.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <deque>
#include <fitplane/GridPlaneInfo.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>  // 特征值求解器
#include <Eigen/SVD>          // SVD分解
#include <Eigen/Core>         // 核心功能

namespace FitPlane
{
// plane.h 中添加的类定义
class IncrementalPlaneFitter {
    private:
        Eigen::Vector3d sum_points_;
        Eigen::Matrix3d sum_outer_products_;
        int point_count_;
        double accumulated_error_;
        bool needs_refinement_;
        
        // SVD拟合的阈值参数
        static constexpr int SVD_REFINEMENT_THRESHOLD = 50;
        static constexpr double ERROR_THRESHOLD = 0.01;
        
    public:
        IncrementalPlaneFitter() : point_count_(0), accumulated_error_(0.0), needs_refinement_(false) {
            sum_points_.setZero();
            sum_outer_products_.setZero();
        }
        
        void reset() {
            sum_points_.setZero();
            sum_outer_products_.setZero();
            point_count_ = 0;
            accumulated_error_ = 0.0;
            needs_refinement_ = false;
        }
        
        void addPoint(const Eigen::Vector3d& point) {
            sum_points_ += point;
            sum_outer_products_ += point * point.transpose();
            point_count_++;
            
            // 检查是否需要SVD精化
            if(point_count_ % SVD_REFINEMENT_THRESHOLD == 0) {
                needs_refinement_ = true;
            }
        }
        
        void addPoints(const std::vector<Eigen::Vector3d>& points) {
            for(const auto& pt : points) {
                addPoint(pt);
            }
        }
        
        // 使用特征值分解的增量拟合
        bool fitPlaneIncremental(Eigen::Vector3d& normal, double& d, double& error) {
            if(point_count_ < 3) return false;
            
            Eigen::Vector3d center = sum_points_ / point_count_;
            Eigen::Matrix3d covariance = sum_outer_products_ / point_count_ - center * center.transpose();
            
            // 使用 Eigen 的特征值分解
            Eigen::EigenSolver<Eigen::Matrix3d> solver(covariance);
            if(solver.info() != Eigen::Success) return false;
            
            // 找到最小特征值对应的特征向量
            Eigen::Vector3cd eigenvalues = solver.eigenvalues();
            Eigen::Matrix3cd eigenvectors = solver.eigenvectors();
            
            int min_eigenvalue_index = 0;
            double min_eigenvalue = eigenvalues(0).real();
            
            for(int i = 1; i < 3; i++) {
                if(eigenvalues(i).real() < min_eigenvalue) {
                    min_eigenvalue = eigenvalues(i).real();
                    min_eigenvalue_index = i;
                }
            }
            
            // 提取实部作为法向量
            normal = eigenvectors.col(min_eigenvalue_index).real();
            
            if(normal.z() < 0) normal = -normal;
            d = -normal.dot(center);
            
            // 计算拟合误差
            error = std::abs(min_eigenvalue);
            accumulated_error_ = error;
            
            return true;
        }
        
        // 使用SVD的精确拟合
        bool fitPlaneSVD(const std::vector<Eigen::Vector3d>& points, 
                         Eigen::Vector3d& normal, double& d, double& error) {
            if(points.size() < 3) return false;
            
            // 计算中心点
            Eigen::Vector3d center = Eigen::Vector3d::Zero();
            for(const auto& pt : points) {
                center += pt;
            }
            center /= points.size();
            
            // 构建矩阵
            Eigen::MatrixXd A(points.size(), 3);
            for(size_t i = 0; i < points.size(); i++) {
                A.row(i) = points[i] - center;
            }
            
            // SVD分解
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
            normal = svd.matrixV().col(2);
            
            if(normal.z() < 0) normal = -normal;
            d = -normal.dot(center);
            
            // 计算拟合误差（最小奇异值的平方）
            Eigen::VectorXd singular_values = svd.singularValues();
            error = singular_values(singular_values.size()-1) * singular_values(singular_values.size()-1) / points.size();
            
            return true;
        }
        
        // 混合拟合方法
        bool fitPlaneHybrid(const std::vector<Eigen::Vector3d>& points,
                           Eigen::Vector3d& normal, double& d, double& error) {
            if(points.empty()) return false;
            
            // 重置并添加所有点
            reset();
            addPoints(points);
            
            Eigen::Vector3d incremental_normal, svd_normal;
            double incremental_d, svd_d;
            double incremental_error, svd_error;
            
            // 增量拟合
            bool incremental_success = fitPlaneIncremental(incremental_normal, incremental_d, incremental_error);
            
            // 决定是否需要SVD精化
            bool use_svd = needs_refinement_ || 
                          incremental_error > ERROR_THRESHOLD ||
                          point_count_ > SVD_REFINEMENT_THRESHOLD;
            
            if(use_svd && incremental_success) {
                // SVD精化
                bool svd_success = fitPlaneSVD(points, svd_normal, svd_d, svd_error);
                
                if(svd_success) {
                    // 比较两种方法的误差，选择更好的结果
                    if(svd_error < incremental_error * 0.8) { // SVD误差显著更小
                        normal = svd_normal;
                        d = svd_d;
                        error = svd_error;
                        needs_refinement_ = false;
                        return true;
                    }
                }
            }
            
            if(incremental_success) {
                normal = incremental_normal;
                d = incremental_d;
                error = incremental_error;
                return true;
            }
            
            return false;
        }
        
        int getPointCount() const { return point_count_; }
        double getAccumulatedError() const { return accumulated_error_; }
        bool needsRefinement() const { return needs_refinement_; }
    };
    
    
const unsigned int ANGLE_BUFFER_SIZE = 10;

struct Plane
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int id = -1;
    Eigen::Vector3d normal_vector;
    std::vector<Eigen::Vector3d> plane_pts;
    Eigen::Vector3d init_coord = Eigen::Vector3d::Zero();
    double traversability=100.0;
    double plane_angle = 0.0;
    double plane_height = 0.0;
}; 

class PlaneMap
{
public:
    PlaneMap(World* world, const float resolution);
    ~PlaneMap();
    bool InitPlaneMap();
    void clearPlaneMap();
    void PointCloudMapCallback(const sensor_msgs::PointCloud2& PointCloud_Map);

    bool init();
    bool getPlaneMap();
    void FitPlane(Plane& result_plane, Eigen::Vector3d& p_surface,World* world,const double &radius);

    void visSurf(const PlaneMap &planemap, ros::Publisher* surf_vis_pub);
    bool pubPlaneGridMap(const PlaneMap &planemap);
    void pubRobotPlaneInfo();
    void pubGridPlaneInfo();

    double getAngle(Eigen::Vector3d &plane_vector);

    Plane** Plane_Map_=NULL; 
    fitplane::PlaneMap planeMapMsg_;
    bool has_PlaneMap_=false;
    float resolution_;
    Eigen::Vector3i index_num_;
    Eigen::Vector3d leftdownbound_;
    Eigen::Vector3d rightupbound_;
    World* world_ = NULL;

    // Parameters for traversability calculation
    float max_angle_; 
    float max_flatness_;
    float w1_;

    std::deque<float> angle_buffer_;
    ros::Publisher plane_grid_map_pub_;
    ros::Publisher robot_plane_info_pub_;
    ros::Publisher grid_plane_info_pub_;

    int width_;
    int height_;
    nav_msgs::OccupancyGrid plane_Occmap_;
    nav_msgs::OccupancyGrid Original_plane_Occmap_;
    ros::Publisher plane_OccMap_pub_;
    ros::Publisher trav_cloud_pub;

    int occThre_ = 100;
    int flatThre_ = 99;
    int nobs_ = 120;
    
    fitplane::GridPlaneInfo grid_plane_info_msg_;

    Eigen::Vector3d index2coord(const Eigen::Vector3i &index) const
    {
        Eigen::Vector3d coord = resolution_*index.cast<double>() + leftdownbound_+ 0.5*resolution_*Eigen::Vector3d::Ones();
        return coord;
    }
    Eigen::Vector3i coord2index(const Eigen::Vector3d &coord)
    {
        Eigen::Vector3i index = ( (coord-leftdownbound_)/resolution_).cast<int>();            
        return index;
    }
    bool isInBorder(const int& x, const int& y)
    {
        return x >= 0 && y >= 0 && x < width_ && y < height_;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr exploredAreaCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    ros::Publisher pubExploredArea;
    double exploredAreaVoxelSize = 0.1;
    pcl::VoxelGrid<pcl::PointXYZ> exploredAreaDwzFilter;
};

}

#endif