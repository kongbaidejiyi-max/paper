
#include "plane.h"
#include <vector>
#include <algorithm>

#define CONTXY2DISC(X, CELLSIZE) (((X) >= 0) ? ((int)((X) / (CELLSIZE))) : ((int)((X) / (CELLSIZE)) - 1))
#define DISCXY2CONT(X, CELLSIZE) ((X) * (CELLSIZE) + (CELLSIZE) / 2.0)

int dx_[4] = {-1, 0, 1, 0};
int dy_[4] = {0, 1, 0, -1};

// FILE* file = fopen("./Analysis.txt", "w+");

namespace FitPlane
{
    PlaneMap::PlaneMap(World* world, const float resolution)
    {
        world_ = world;
        resolution_ = resolution;
        init();
    }

    void PlaneMap::PointCloudMapCallback(const sensor_msgs::PointCloud2& PointCloud_Map)
    {
        // Slam-sim-out or Fast-Lio
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(PointCloud_Map, cloud);
        if (!world_->initGridMap(cloud))
        {
            ROS_WARN_THROTTLE(1.0, "World map initialization failed, likely due to empty point cloud. Skipping frame.");
            return;
        }

        for (const auto& pt : world_->cloud_near_)
        {
            Eigen::Vector3d obstacle(pt.x, pt.y, pt.z);
            world_->setObs(obstacle);
        }
        // visualization::visWorld(world_, &world_->Grid_Map_pub); 
        InitPlaneMap();
        
        timeval start;
        gettimeofday(&start, NULL);
        
        getPlaneMap();
        pubPlaneGridMap(*this);
        pubRobotPlaneInfo();
        pubGridPlaneInfo();
        timeval end;
        gettimeofday(&end, NULL);
        // double ms = 1000 * (end.tv_sec - start.tv_sec) + 0.001 * (end.tv_usec - start.tv_usec);
        // fprintf(file, "%lf \n", ms);
    }

    bool PlaneMap::init()
    {
        ros::NodeHandle nh("~");
        nh.param("max_angle", max_angle_, 40.0f);
        nh.param("max_flatness", max_flatness_, 0.7f);
        nh.param("w1", w1_, 0.8f);

        ROS_INFO("Loaded plane fitting params: max_angle=%.2f, max_flatness=%.2f, w1=%.2f", max_angle_, max_flatness_, w1_);

        world_->PointCloud_Map_sub = world_->nh_.subscribe(world_->PointCloud_Map_topic, 10, &FitPlane::PlaneMap::PointCloudMapCallback, this);

        plane_OccMap_pub_ = world_->nh_.advertise<nav_msgs::OccupancyGrid>("plane_OccMap", 1);
        trav_cloud_pub = world_->nh_.advertise<sensor_msgs::PointCloud2>("local_traversibility_ponit_cloud", 1);
        
        // 添加机器人所在平面信息发布器
        robot_plane_info_pub_ = world_->nh_.advertise<fitplane::PlaneMap>("robot_plane_info", 1);

        // 添加新的发布器
        grid_plane_info_pub_ = world_->nh_.advertise<fitplane::GridPlaneInfo>("grid_plane_info", 1);

        return true;        
    }

    PlaneMap::~PlaneMap()
    {
        clearPlaneMap();
    }
    
    void PlaneMap::clearPlaneMap()
    {
        if(this->has_PlaneMap_)
        {
            for(int i=0;i < index_num_(0);i++)
            {
                if(Plane_Map_[i] != NULL)
                {
                    delete[] Plane_Map_[i]; 
                    Plane_Map_[i]=NULL; 
                }
            }
            if(Plane_Map_ !=NULL)
            {
                delete[] Plane_Map_;
                Plane_Map_=NULL;
            }            
        }

    }

    bool PlaneMap::InitPlaneMap()
    {
        if(this->has_PlaneMap_)
            clearPlaneMap();

        if(world_ != NULL)
        {
            leftdownbound_ = world_->getLowerBound();
            rightupbound_ = world_->getUpperBound();
            index_num_ = ((rightupbound_ - leftdownbound_) / resolution_).cast<int>() + Eigen::Vector3i::Ones(); 
            Plane_Map_ = new Plane*[index_num_(0)];
            for(int i = 0; i < index_num_(0); i++)
            {
                Plane_Map_[i] = new Plane[index_num_(1)];
            }
            this->has_PlaneMap_ = true; 
        }
        else
        {
            ROS_ERROR("No world ! ");
            this->has_PlaneMap_ = false;
        }

        return this->has_PlaneMap_;
    }

    bool PlaneMap::getPlaneMap()
    {
        int r_x = CONTXY2DISC(world_->ego_position_.x - leftdownbound_(0), resolution_); 
        int r_y = CONTXY2DISC(world_->ego_position_.y - leftdownbound_(1), resolution_);
        int lowerx = std::max(0, r_x - int(20.0 / resolution_));
        int lowery = std::max(0, r_y - int(20.0 / resolution_));
        int upperx = std::min(index_num_(0), r_x + int(20.0 / resolution_));
        int uppery = std::min(index_num_(1), r_y + int(20.0 / resolution_));
        for(int i = lowerx; i < upperx; i++)
        {
            for(int j = lowery; j < uppery; j++)
            {
                Eigen::Vector3d tmp_P(PlaneMap::index2coord(Eigen::Vector3i(i,j,0)));
                Eigen::Vector3d p_sur;
                if(world_->project2surface(tmp_P(0), tmp_P(1), &p_sur))
                {
                    FitPlane(Plane_Map_[i][j], p_sur, world_,0.5);
                }
            }
        }
        pubPlaneGridMap(*this);
        pubRobotPlaneInfo();
        pubGridPlaneInfo();
        return true;
    }

void PlaneMap::FitPlane(Plane& result_plane, Eigen::Vector3d& p_surface, World* world, const double &radius)
{
    static constexpr int MAX_Z_RANGE = 10;
    static constexpr int MIN_POINTS_THRESHOLD = 10;
    static constexpr double UNUSABLE_TRAVERSABILITY = 100.0;
    
    // 重置结果
    result_plane = Plane();
    result_plane.init_coord = p_surface;        

    const Eigen::Vector3d ball_center = world->coordRounding(p_surface);
    const float resolution = world->getResolution();
    const int fit_num = static_cast<int>(radius/resolution);
    const int dim = 2 * fit_num + 1;
    
    // --- 1. 在一个三维邻域内收集所有点 ---
    std::vector<Eigen::Vector3d> all_points_in_volume;
    all_points_in_volume.reserve(1000);
    
    // 点收集循环
    for(int i = -fit_num; i <= fit_num; i++) {
        for(int j = -fit_num; j <= fit_num; j++) {
            for(int k = -MAX_Z_RANGE; k <= MAX_Z_RANGE; k++) { 
                const Eigen::Vector3d point = ball_center + resolution * Eigen::Vector3d(i, j, k); 
                
                if(world->isInsideBorder(point) && !world->isFree(point)) {
                    all_points_in_volume.push_back(point);
                    if(all_points_in_volume.size() >= 1000) goto collection_done;
                }
            }
        }
    }
    
collection_done:
    // --- 2. 关键步骤：过滤收集到的点，分离地面与高处障碍物 ---
    if (all_points_in_volume.size() < MIN_POINTS_THRESHOLD) {
        result_plane.traversability = UNUSABLE_TRAVERSABILITY;
        return;
    }

    // 使用初始地面种子点 p_surface 的高度作为基准
    const double ground_z_ref = p_surface.z();
    // 定义一个地面厚度阈值。远高于此值的点被视为悬空障碍物（如杆子）
    const double max_ground_thickness = 0.20; // 20厘米

    result_plane.plane_pts.reserve(all_points_in_volume.size());
    for (const auto& pt : all_points_in_volume) {
        // 只保留那些高度与地面基准相近的点
        if (pt.z() < ground_z_ref + max_ground_thickness) {
            result_plane.plane_pts.push_back(pt);
        }
    }

    // --- 3. 使用清洗后的纯地面点进行拟合 ---
    const size_t pt_num = result_plane.plane_pts.size();
    if(pt_num < MIN_POINTS_THRESHOLD) {
        // 如果过滤后地面点太少，则无法拟合
        result_plane.traversability = UNUSABLE_TRAVERSABILITY;
        return;
    }

    // 创建增量式拟合器
    IncrementalPlaneFitter fitter;
    Eigen::Vector3d normal;
    double plane_d;
    double fitting_error;
    
    // 使用混合拟合方法
    if(!fitter.fitPlaneHybrid(result_plane.plane_pts, normal, plane_d, fitting_error)) {
        result_plane.traversability = UNUSABLE_TRAVERSABILITY;
        return;
    }
    
    result_plane.normal_vector = normal;
    
    // 计算平面参数
    const double angle_rad = getAngle(normal);
    const double angle_deg = angle_rad * 180.0 / PI;
    
    result_plane.plane_angle = angle_deg;
    result_plane.plane_height = (-plane_d) / std::max(0.001, std::abs(normal.z())); // 避免除零
    
    // 计算平整度
    double flatness = 0.0;
    for(const auto& pt : result_plane.plane_pts) {
        double distance = std::abs(normal.dot(pt) + plane_d);
        flatness += distance * distance;
    }
    flatness = std::sqrt(flatness / pt_num); // RMS距离
    
    // 可通行性评估
    if(angle_deg >= max_angle_ || flatness >= max_flatness_) {
        result_plane.traversability = 1.0;
    } else {
        double angle_factor = angle_deg / max_angle_;
        double flatness_factor = flatness / max_flatness_;
        
        result_plane.traversability = w1_ * angle_factor + (1.0 - w1_) * flatness_factor;
        result_plane.traversability = std::min(1.0, result_plane.traversability);
    }
}

    bool PlaneMap::pubPlaneGridMap(const PlaneMap &planemap)
    {
        plane_Occmap_.info.height = planemap.index_num_(1) * int(resolution_ / resolution_); // 0.5
        plane_Occmap_.info.width = planemap.index_num_(0) * int(resolution_ / resolution_);// 0.5
        plane_Occmap_.info.resolution = resolution_;// 0.5
        plane_Occmap_.header.frame_id = "map";
        plane_Occmap_.info.origin.position.x = leftdownbound_(0);
        plane_Occmap_.info.origin.position.y = leftdownbound_(1);
        plane_Occmap_.data.clear();
        std::vector<int8_t> tmp(plane_Occmap_.info.height * plane_Occmap_.info.width, -1);
        plane_Occmap_.data = tmp;

        for(unsigned int i = 0; i < plane_Occmap_.info.width; i++)
        {
            for(unsigned int j = 0; j < plane_Occmap_.info.height; j++)
            {   
                double x = DISCXY2CONT(i, resolution_) + leftdownbound_(0);// 0.5
                double y = DISCXY2CONT(j, resolution_) + leftdownbound_(1);// 0.5

                int l_x = CONTXY2DISC(x - leftdownbound_(0), resolution_); 
                int l_y = CONTXY2DISC(y - leftdownbound_(1), resolution_);
            
                if(l_x >= 0 && l_x < planemap.index_num_(0) && l_y >= 0 && l_y < planemap.index_num_(1))
                {
                    if(planemap.Plane_Map_[l_x][l_y].traversability != 100)
                    {
                        if(planemap.Plane_Map_[l_x][l_y].traversability == 1)// 0.9 && planemap.Plane_Map_[i][j].traversability <= 1.0)
                        {
                            plane_Occmap_.data[i + plane_Occmap_.info.width * j]= occThre_;
                        }
                        else if(planemap.Plane_Map_[l_x][l_y].traversability == 10)// 0.9 && planemap.Plane_Map_[i][j].traversability <= 1.0)
                        {
                            plane_Occmap_.data[i + plane_Occmap_.info.width * j]= nobs_;
                        }
                        else
                            plane_Occmap_.data[i + plane_Occmap_.info.width * j] = 100 * planemap.Plane_Map_[l_x][l_y].traversability;   

                    }
                }


            }
        }

        Original_plane_Occmap_ = plane_Occmap_;
        int d_x8[8] = {-1, 0, 0, 1, 1, -1, 1, -1};
        int d_y8[8] = {0, 1, -1, 0, 1, -1, -1, 1};
        width_ = plane_Occmap_.info.width;
        height_ = plane_Occmap_.info.height;
        int sum = 0;
        int count = 0;
        for(int i = 0; i < width_; i++)
        {
            for(int j = 0; j < height_; j++)
            {

                if(Original_plane_Occmap_.data[i + j * width_] == occThre_) 
                {
                    for(int d = 0; d < 8; d++)
                    {
                        int x_tmp = i + d_x8[d];
                        int y_tmp = j + d_y8[d];
                        if(isInBorder(x_tmp, y_tmp))
                        {
                            if(Original_plane_Occmap_.data[x_tmp + y_tmp * width_] == -1)
                            {
                                sum = sum + 0;
                                count++;
                            }
                            else if(Original_plane_Occmap_.data[x_tmp + y_tmp * width_] != occThre_)
                            {
                                sum = Original_plane_Occmap_.data[x_tmp + y_tmp * width_] + sum;
                                count++;
                            }
                        }
                    }
                    if(count == 8)
                    {
                        plane_Occmap_.data[i + j * width_] = (plane_Occmap_.data[i + j * width_] + sum ) / (count + 1);
                    }
                    sum = 0;    
                    count = 0;                    
                }
            }
        }
        for(int i = 0; i < width_; i++)
        {
            for(int j = 0; j < height_; j++)
            {
                if(plane_Occmap_.data[i + j * width_] == occThre_ || plane_Occmap_.data[i + j * width_] == nobs_) // 
                {
                    for(int d = 0; d < 8; d++)
                    {
                        int x_tmp = i + d_x8[d];
                        int y_tmp = j + d_y8[d];
                        if(isInBorder(x_tmp, y_tmp))
                        {
                            if(plane_Occmap_.data[x_tmp + y_tmp * width_] < flatThre_)
                            {
                                if(plane_Occmap_.data[i + j * width_] == occThre_)
                                    plane_Occmap_.data[x_tmp + y_tmp * width_] = flatThre_;
                                else
                                {
                                    plane_Occmap_.data[x_tmp + y_tmp * width_] =  nobs_ - 1;
                                }
                            }
                        }
                    }
                } 
            }
        }
        plane_Occmap_.header.stamp = ros::Time::now();
        plane_OccMap_pub_.publish(plane_Occmap_);

        int r_x = CONTXY2DISC(world_->ego_position_.x - leftdownbound_(0), resolution_); 
        int r_y = CONTXY2DISC(world_->ego_position_.y - leftdownbound_(1), resolution_);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr trav_point_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        
        // 可视化范围 - 使用与pubRobotPlaneInfo相同的范围
        int grid_range = int(1.0 / resolution_);
        int vis_range = 12; // 可视化范围保持为12，以保持较好的视觉效果
        
        // 获取机器人周围平面的平均角度，用于颜色映射
        float avg_angle = 0.0f;
        int valid_count = 0;
        
        // 首先计算平均角度，用于后续的颜色映射
        for(int i = r_x - grid_range; i < r_x + grid_range + 1; i++)
        {
            for(int j = r_y - grid_range; j < r_y + grid_range + 1; j++)
            {
                if(i >= 0 && i < index_num_(0) && j >= 0 && j < index_num_(1))
                {
                    if(Plane_Map_[i][j].traversability != 100)
                    {
                        avg_angle += Plane_Map_[i][j].plane_angle;
                        valid_count++;
                    }
                }
            }
        }
        
        if(valid_count > 0)
        {
            avg_angle /= valid_count;
            // ROS_INFO("Average angle in visualization area: %.2f degrees", avg_angle);
        }
        
        // 可视化更大范围的平面
        for(int i = r_x - vis_range; i < r_x + vis_range + 1; i++)
        {
            for(int j = r_y - vis_range; j < r_y + vis_range + 1; j++)
            {
                if(isInBorder(i, j))
                {
                    double x = DISCXY2CONT(i, resolution_) + leftdownbound_(0);
                    double y = DISCXY2CONT(j, resolution_) + leftdownbound_(1);
                    
                    pcl::PointXYZRGB reg_point;
                    reg_point.x = x;
                    reg_point.y = y;
                    
                    // 检查是否在机器人所在平面范围内（1m）
                    bool is_in_robot_plane_range = (i >= r_x - grid_range && i <= r_x + grid_range && 
                                                   j >= r_y - grid_range && j <= r_y + grid_range);
                    
                    if(plane_Occmap_.data[i + j * width_] <= 99)
                    {
                        // 可通行区域 - 绿色
                        reg_point.z = world_->ego_position_.z + 0.1;
                        reg_point.r = 0;
                        reg_point.g = 255;
                        reg_point.b = 0;
                        
                        // 如果在机器人所在平面范围内，使用不同的颜色
                        if(is_in_robot_plane_range)
                        {
                            reg_point.z = world_->ego_position_.z + 0.2; // 稍微抬高以便区分
                            reg_point.r = 0;
                            reg_point.g = 255;
                            reg_point.b = 255; // 青色
                        }
                    }
                    else
                    {
                        // 不可通行区域 - 红色
                        reg_point.z = world_->ego_position_.z - 0.1;
                        reg_point.r = 255;
                        reg_point.g = 0;
                        reg_point.b = 0;
                    }
                    
                    // 如果是机器人位置，使用特殊颜色标记
                    if(i == r_x && j == r_y)
                    {
                        reg_point.z = world_->ego_position_.z + 0.3; // 更高以便清晰可见
                        reg_point.r = 255;
                        reg_point.g = 255;
                        reg_point.b = 0; // 黄色
                    }
                    
                    trav_point_cloud->points.push_back(reg_point);
                }
            }
        }
        
        
        trav_point_cloud->width = trav_point_cloud->points.size();
        trav_point_cloud->height = 1;
        trav_point_cloud->is_dense = true;
        
        sensor_msgs::PointCloud2 trav_point_cloud_msg;
        pcl::toROSMsg(*trav_point_cloud, trav_point_cloud_msg);
        trav_point_cloud_msg.header.stamp = ros::Time::now();
        trav_point_cloud_msg.header.frame_id = "map";
        trav_cloud_pub.publish(trav_point_cloud_msg);

        return true;
    }

    double PlaneMap::getAngle(Eigen::Vector3d &plane_vector)
    {
        Eigen::Vector3d n1(0,0,1);
	    double cos_ = abs(n1(0) * plane_vector(0) + n1(1) * plane_vector(1) + n1(2) * plane_vector(2)) / 
			    (sqrt(n1(0) * n1(0) + n1(1) * n1(1) + n1(2) * n1(2)) * 
         		sqrt(plane_vector(0) * plane_vector(0) + plane_vector(1) * 
	    		plane_vector(1) + plane_vector(2) * plane_vector(2)));
        double angle = std::acos(cos_);
	return angle;
    }

    void PlaneMap::pubRobotPlaneInfo()
    {
        if (!has_PlaneMap_)
            return;

        // --- 1. 获取机器人当前状态 ---
        Eigen::Vector3d robot_pos(world_->ego_position_.x, world_->ego_position_.y, world_->ego_position_.z);
        Eigen::Quaterniond robot_orientation(world_->ego_orientation_.w, 
                                            world_->ego_orientation_.x, 
                                            world_->ego_orientation_.y, 
                                            world_->ego_orientation_.z);

        // 获取机器人正下方的地面高度，作为后续相对高度计算的参考基准
        Eigen::Vector3d ground_under_robot;
        double robot_ground_z = 0.0;
        if (world_->project2surface(robot_pos(0), robot_pos(1), &ground_under_robot))
        {
            robot_ground_z = ground_under_robot(2);
        }
        else
        {
            // 如果正下方没有地面，则使用机器人自己的Z坐标作为备用参考
            robot_ground_z = robot_pos(2);
            // ROS_WARN_THROTTLE(1.0, "无法找到机器人下方的地面，使用机器人Z高度作为参考");
        }

        // --- 2. 寻找用于分析的地面点 ---
        Eigen::Vector3d p_sur;
        bool surface_found = false;

        // 定义一个辅助函数，用于在指定方向和距离上搜索地面点
        auto find_surface = [&](const Eigen::Vector3d& direction, double target_dist, double search_window) -> bool 
        {
            // 在目标距离附近的一个窗口内进行搜索，以提高成功率
            for (double dist_offset = -search_window / 2.0; dist_offset <= search_window / 2.0; dist_offset += 0.1)
            {
                double current_dist = target_dist + dist_offset;
                if (current_dist <= 0) continue;

                // 在机器人前方进行横向扫描
                for (double lateral = 0; lateral <= 0.5; lateral += 0.1)
                {
                    Eigen::Vector3d lateral_offset = robot_orientation * Eigen::Vector3d(0.0, lateral, 0.0);
                    Eigen::Vector3d search_pos = robot_pos + direction * current_dist + lateral_offset;
                    
                    if (world_->project2surface(search_pos(0), search_pos(1), &p_sur))
                    {
                        // ROS_INFO("在距离%.2fm, 横向%.2fm处找到地面点", current_dist, lateral);
                        return true;
                    }
                }
            }
            return false;
        };

        // 优先搜索: 机器人前方4.3米
        const Eigen::Vector3d front_direction = robot_orientation * Eigen::Vector3d(1.0, 0.0, 0.0);
        surface_found = find_surface(front_direction, 4.8, 1.0); // 在4.3m距离附近 +/- 0.5m的窗口内搜索

        // 备用方案: 机器人后方1米
        if (!surface_found)
        {
            // ROS_WARN("前方未找到地面点，正在尝试搜索机器人后方1m处...");
            const Eigen::Vector3d back_direction = robot_orientation * Eigen::Vector3d(-1.0, 0.0, 0.0);
            surface_found = find_surface(back_direction, 2.0, 1); // 在1m距离附近 +/- 0.25m的窗口内搜索
        }

        if (!surface_found)
        {
            // ROS_WARN("在机器人前方或后方均未找到可分析的地面点");
            return;
        }
        
        // --- 3. 对找到的地面点进行平面拟合 ---
        Plane target_plane;
        target_plane.traversability = 100.0; // 初始状态：拟合失败

        double used_radius = -1.0;
        const double MIN_FIT_RADIUS = 0.5;
        const double MAX_FIT_RADIUS = 1.0;
        const double FIT_RADIUS_STEP = 0.05;

        // 从小到大尝试不同的拟合半径，直到成功
        for (double radius = MIN_FIT_RADIUS; radius <= MAX_FIT_RADIUS; radius += FIT_RADIUS_STEP)
        {
            FitPlane(target_plane, p_sur, world_, radius);
            if (target_plane.traversability != 100)
            {
                used_radius = radius;
                // ROS_INFO("使用半径 %.2f 米成功拟合平面", used_radius);
                break; 
            }
        }

        // --- 4. 处理并发布结果 ---
        if (target_plane.traversability == 100)
        {
            // ROS_WARN("即使使用最大半径，也无法在目标位置(%.2f, %.2f, %.2f)拟合出有效平面", 
                    //  p_sur(0), p_sur(1), p_sur(2));
            return;
        }

        // 如果最终使用的拟合半径过小，结果可能不稳定，为安全起见将其视为平地
        const double min_stable_radius = 0.1;
        if (used_radius > 0 && used_radius < min_stable_radius)
        {
            // ROS_INFO("因使用较小拟合半径(%.2f m)，为保证稳定，将地面视为平坦", used_radius);
            // target_plane.plane_angle = 0.0;
        }

        float final_angle = target_plane.plane_angle;

        // 使用滑动平均滤波器对角度进行平滑处理，以减少抖动
        angle_buffer_.push_back(final_angle);
        if (angle_buffer_.size() > ANGLE_BUFFER_SIZE)
        {
            angle_buffer_.pop_front();
        }
        
        float smoothed_angle = 0.0f;
        if (!angle_buffer_.empty())
        {
            for (float angle : angle_buffer_)
            {
                smoothed_angle += angle;
            }
            smoothed_angle /= angle_buffer_.size();
        }

        // 计算找到的平面的高度与机器人正下方地面的相对高度
        float relative_height = target_plane.plane_height - robot_ground_z;
        // ROS_INFO("分析平面: 角度=%.2f (平滑后=%.2f) 度, 相对高度=%.2f 米", 
        //         final_angle, smoothed_angle, relative_height);

        // 填充并发布平面信息消息
        planeMapMsg_.width = index_num_(0);
        planeMapMsg_.height = index_num_(1);
        planeMapMsg_.resolution = resolution_;
        planeMapMsg_.origin_x = leftdownbound_(0);
        planeMapMsg_.origin_y = leftdownbound_(1);
        
        planeMapMsg_.PlaneGridMap.clear();
        
        fitplane::Plane plane_msg;
        plane_msg.PlaneCellHeight = relative_height;
        plane_msg.PlaneCellAngle = smoothed_angle; // 使用平滑后的角度
        
        planeMapMsg_.PlaneGridMap.push_back(plane_msg);
        
        robot_plane_info_pub_.publish(planeMapMsg_);
    }

    void PlaneMap::pubGridPlaneInfo()
    {
        if (!has_PlaneMap_)
            return;

        grid_plane_info_msg_.header.frame_id = "map";
        grid_plane_info_msg_.header.stamp = ros::Time::now();
        grid_plane_info_msg_.resolution = resolution_;
        grid_plane_info_msg_.width = index_num_(0);
        grid_plane_info_msg_.height = index_num_(1);
        grid_plane_info_msg_.origin.x = leftdownbound_(0);
        grid_plane_info_msg_.origin.y = leftdownbound_(1);
        grid_plane_info_msg_.origin.z = leftdownbound_(2);

        // 清空并预分配数组大小
        size_t total_size = index_num_(0) * index_num_(1);
        grid_plane_info_msg_.traversability.clear();
        grid_plane_info_msg_.plane_angle.clear();
        grid_plane_info_msg_.plane_height.clear();
        grid_plane_info_msg_.occupancy.clear();
        grid_plane_info_msg_.traversability.resize(total_size);
        grid_plane_info_msg_.plane_angle.resize(total_size);
        grid_plane_info_msg_.plane_height.resize(total_size);
        grid_plane_info_msg_.occupancy.resize(total_size);

        // 填充数据
        for(int i = 0; i < index_num_(0); i++)
        {
            for(int j = 0; j < index_num_(1); j++)
            {
                size_t idx = i + j * index_num_(0);
                if(Plane_Map_[i][j].traversability != 100.0)
                {
                    grid_plane_info_msg_.traversability[idx] = Plane_Map_[i][j].traversability;
                    grid_plane_info_msg_.plane_angle[idx] = Plane_Map_[i][j].plane_angle;
                    grid_plane_info_msg_.plane_height[idx] = Plane_Map_[i][j].plane_height;
                    
                    // 根据traversability设置occupancy
                    if(Plane_Map_[i][j].traversability >= 0.9)
                    {
                        grid_plane_info_msg_.occupancy[idx] = 100;  // 不可通行
                    }
                    else
                    {
                        grid_plane_info_msg_.occupancy[idx] = 0;    // 可通行
                    }
                }
                else
                {
                    // 未知区域
                    grid_plane_info_msg_.traversability[idx] = -1;
                    grid_plane_info_msg_.plane_angle[idx] = -1;
                    grid_plane_info_msg_.plane_height[idx] = -1;
                    grid_plane_info_msg_.occupancy[idx] = -1;
                }
            }
        }

        grid_plane_info_pub_.publish(grid_plane_info_msg_);
    }

}
