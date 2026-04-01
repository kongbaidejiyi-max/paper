
#include "plane.h"

using namespace std;
using namespace Eigen;

namespace FitPlane
{

World::World(const float &resolution, const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
{
    lowerbound_=INF*Vector3d::Ones(); 
    upperbound_=-INF*Vector3d::Ones();
    idx_count_=Vector3i::Zero(); 
    resolution_ = resolution;
    nh_ = nh;
    nh_private_ = nh_private;

    nh_private_.getParam("PointCloud_Map_topic", PointCloud_Map_topic);
    nh_private_.getParam("Grid_Map_topic", Grid_Map_topic);
    nh_private_.getParam("PointCloud_topic", PointCloud_topic);
    nh_private_.getParam("robot_pose_topic", robot_pose_topic);
    
    nh_private_.getParam("use_ex_range", use_ex_range_);
    nh_private_.getParam("ex_robot_back", ex_robot_back_);
    nh_private_.getParam("ex_robot_front", ex_robot_front_);
    nh_private_.getParam("ex_robot_right", ex_robot_right_);
    nh_private_.getParam("ex_robot_left", ex_robot_left_);
    
    Grid_Map_pub = nh_.advertise<sensor_msgs::PointCloud2>(Grid_Map_topic, 1);
    
    // 订阅机器人位置话题
    robot_pose_sub = nh_.subscribe(robot_pose_topic, 10, &World::RobotPoseCallback, this);
}

World::~World()
{
    clearMap();
}

void World::clearMap()
{
    if(has_map_)
    {
        for(int i=0;i < idx_count_(0);i++)
        {
            for(int j=0;j < idx_count_(1);j++)
            {
                delete[] grid_map_[i][j]; 
                grid_map_[i][j]=NULL; 
            }
            delete[] grid_map_[i];
            grid_map_[i]=NULL;
        }
        delete[] grid_map_;
        grid_map_=NULL;
    }
}

bool World::initGridMap(const Vector3d &lowerbound,const Vector3d &upperbound)
{
    lowerbound_=lowerbound;
    upperbound_=upperbound;
    idx_count_=((upperbound_-lowerbound_)/resolution_).cast<int>()+Eigen::Vector3i::Ones(); 
    grid_map_=new bool**[idx_count_(0)];
    for(int i=0;i < idx_count_(0);i++)
    {
        grid_map_[i]=new bool*[idx_count_(1)];
        for(int j=0;j < idx_count_(1);j++)
        {
            grid_map_[i][j]=new bool[idx_count_(2)];
            memset(grid_map_[i][j],true,idx_count_(2)*sizeof(bool)); 
        }
    }
    has_map_=true; 
    return true;
}

bool World::initGridMap(const pcl::PointCloud<pcl::PointXYZ> &cloud)
{   
    if(cloud.points.empty())
    {
        ROS_ERROR("Can not initialize the map with an empty point cloud!");
        return false;
    }
    clearMap();
    cloud_near_.clear();
    for(const auto&pt:cloud.points)
    {
        if(use_ex_range_)
        {
            if(pt.x < ego_position_.x + ex_robot_back_ || pt.x > ego_position_.x + ex_robot_front_ ||
               pt.y < ego_position_.y + ex_robot_right_ || pt.y > ego_position_.y + ex_robot_left_)
                continue;            
        }
        if(abs(pt.x - ego_position_.x) > minrange_ || abs(pt.y - ego_position_.y) > minrange_)
            continue;
        else
        {
            cloud_near_.points.push_back(pt);        
        }
    }

    if (cloud_near_.points.empty())
    {
        ROS_WARN("After filtering, the near point cloud is empty. Skipping map update for this frame.");
        return false;
    }

    float minx = 10000;
    float miny = 10000;
    float minz = 10000; 
    float maxx = -10000;
    float maxy = -10000;
    float maxz = -10000;    
    for(const auto&pt:cloud_near_.points)
    {
        if(pt.x < minx)
        {
            minx = pt.x;
            lowerbound_(0)=minx;
        } 
        if(pt.y < miny)
        {
            miny = pt.y;
            lowerbound_(1)=miny;
        } 
        if(pt.z < minz)
        {
            minz = pt.z;
            lowerbound_(2)=minz;
        } 

        if(pt.x > maxx)
        {
            maxx = pt.x;
            upperbound_(0)=maxx;
        } 
        if(pt.y > maxy)
        {
            maxy = pt.y;
            upperbound_(1)=maxy;
        } 
        if(pt.z + 1.0 > maxz)
        {
            maxz = pt.z + 1;
            upperbound_(2)=maxz;
        } 
    }

    idx_count_ = ((upperbound_-lowerbound_)/resolution_).cast<int>() + Eigen::Vector3i::Ones();

    grid_map_=new bool**[idx_count_(0)];
    for(int i = 0 ; i < idx_count_(0) ; i++)
    {
        grid_map_[i]=new bool*[idx_count_(1)];
        for(int j = 0 ; j < idx_count_(1) ; j++)
        {
            grid_map_[i][j]=new bool[idx_count_(2)];
            memset(grid_map_[i][j],true,idx_count_(2)*sizeof(bool));
        }
    }
    has_map_=true;
    return true;
}

void World::setObs(const Vector3d &point)
{   
    Vector3i idx=coord2index(point);
    grid_map_[idx(0)][idx(1)][idx(2)]=false; 
}

bool World::isFree(const Vector3d &point)
{
    Vector3i idx = coord2index(point);
    bool is_free = isInsideBorder(idx) && grid_map_[idx(0)][idx(1)][idx(2)];
    return is_free;
}

Vector3d World::coordRounding(const Vector3d & coord)
{
    return index2coord(coord2index(coord));
}

bool World::project2surface(const float &x,const float &y,Vector3d* p_surface)
{
    // A more robust method to find the ground surface by analyzing the entire vertical column of points.
    // This helps to ignore "hanging" points from vegetation or noise.
    if(!(x>=lowerbound_(0) && x<=upperbound_(0) && y>=lowerbound_(1) && y<=upperbound_(1)))
    {
        return false;
    }

    std::vector<double> z_points;
    // 1. Collect all occupied Z-coordinates in the vertical column.
    for(float z = lowerbound_(2); z <= upperbound_(2) ; z+=resolution_)
    {
        if( !isFree(x,y,z) )
        {
            z_points.push_back(z);
        }
    }

    if(z_points.empty())
    {
        return false;
    }
    
    // If there is only one point, it's the surface.
    if (z_points.size() == 1) {
        *p_surface = Vector3d(x, y, z_points[0]);
        return true;
    }

    // 2. Find the largest vertical gap between points.
    double max_gap = 0.0;
    int ground_cluster_end_index = 0; 
    // A reasonable gap (e.g., 30cm) to distinguish ground from an obstacle/vegetation above it.
    const double MIN_OBSTACLE_HEIGHT_GAP = 0.3; 

    for (size_t i = 0; i < z_points.size() - 1; ++i)
    {
        double current_gap = z_points[i+1] - z_points[i];
        if (current_gap > max_gap)
        {
            max_gap = current_gap;
            // 3. If a significant gap is found, we assume everything below it is the ground cluster.
            //    The highest point of this cluster is our best candidate for the ground surface.
            if (max_gap > MIN_OBSTACLE_HEIGHT_GAP) {
                 ground_cluster_end_index = i;
                 // We break here because we only care about the first major gap from the bottom up.
                 break; 
            }
        }
    }

    // If no significant gap was found (e.g., a continuous ramp or wall), 
    // assume all points belong to the same surface cluster. The highest point is the surface.
    if (max_gap <= MIN_OBSTACLE_HEIGHT_GAP) {
        ground_cluster_end_index = z_points.size() - 1;
    }

    // 4. Return the highest point of the identified ground cluster.
    *p_surface = Vector3d(x, y, z_points[ground_cluster_end_index]);
    return true;
}

bool World::isInsideBorder(const Vector3i &index)
{
    return index(0) >= 0 &&
           index(1) >= 0 &&
           index(2) >= 0 && 
           index(0) < idx_count_(0)&&
           index(1) < idx_count_(1)&&
           index(2) < idx_count_(2);
}

// 实现机器人位置回调函数
void World::RobotPoseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    ego_position_ = msg->pose.pose.position;
    // ROS_INFO("Robot position: x=%.2f, y=%.2f, z=%.2f", ego_position_.x, ego_position_.y, ego_position_.z);
    ego_orientation_ = msg->pose.pose.orientation;
}
}

