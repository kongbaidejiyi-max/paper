# ROS 协同导航仿真项目

这是一个基于 ROS Noetic 的多机器人协同导航仿真项目，实现了地面车辆（UGV）和无人机（UAV）的自主导航、路径规划和可视化功能。

## 目录结构

```
.
├── Car/                          # 车辆工作空间
│   ├── src/
│   │   ├── ego-planner/          # EGO-Planner 规划器
│   │   │   ├── src/planner/
│   │   │   │   ├── plan_manage/  # 规划管理模块
│   │   │   │   ├── plan_env/     # 环境感知模块
│   │   │   │   ├── path_searching/  # 路径搜索
│   │   │   │   ├── bspline_opt/  # B样条轨迹优化
│   │   │   │   └── traj_utils/   # 轨迹工具
│   │   │   └── src/uav_simulator/ # 仿真器模块
│   │   ├── car_visualization/    # 车辆可视化包
│   │   └── mpc/                  # MPC 控制器
│   └── build/, devel/
│
├── Drone/                        # 无人机工作空间
│   ├── src/
│   │   ├── fitplane/             # 平面拟合与可通行性分析
│   │   ├── fitplane_planner/     # 路径规划器（A* + GRP）
│   │   └── drone_visualization/  # 无人机可视化包
│   └── build/, devel/
│
├── launch_all.sh                 # 一键启动脚本
└── stop_all.sh                   # 一键关闭脚本
```

## 功能模块

### Car 工作空间

| 模块 | 功能 | 启动命令 |
|------|------|----------|
| **ego_planner** | 车辆自主导航规划器，支持 GRP 模式 | `roslaunch ego_planner grp_run.launch` |
| **car_visualization** | 在 RViz 中可视化车辆模型和轨迹 | `roslaunch car_visualization car_visualization.launch` |
| **mpc_controller** | MPC 轨迹跟踪控制器 | - |
| **transform.py** | 将 OccupancyGrid 转换为 3D 点云 | `python3 transform.py` |

### Drone 工作空间

| 模块 | 功能 | 启动命令 |
|------|------|----------|
| **fitplane** | 平面检测与可通行性分析，生成可通行地图 | `roslaunch fitplane sim.launch` |
| **fitplane_planner** | 基于 A* 的全局路径规划器 | `roslaunch fitplane_planner sim.launch` |
| **drone_visualization** | 在 RViz 中可视化无人机模型 | `roslaunch drone_visualization drone_visualization.launch` |
| **acc_point.py** | 点云累积器（滑动窗口融合） | `python3 acc_point.py` |

## ROS 话题接口

### 输入话题
| 话题名称 | 消息类型 | 说明 |
|----------|----------|------|
| `/car/Odometry` | `nav_msgs/Odometry` | 车辆里程计 |
| `/car/cloud_registered` | `sensor_msgs/PointCloud2` | 车辆配准点云 |
| `/fused_map` | `nav_msgs/OccupancyGrid` | 融合后的占据栅格地图 |

### 输出话题
| 话题名称 | 消息类型 | 说明 |
|----------|----------|------|
| `/grp` | `nav_msgs/Path` | 全局参考路径 |
| `/planning/bspline` | `bspline/Bspline` | B样条轨迹 |
| `/car/cmd_vel` | `geometry_msgs/Twist` | 车辆速度控制指令 |
| `/cloud_accumulateds` | `sensor_msgs/PointCloud2` | 累积点云 |
| `/occupancy_point_cloud` | `sensor_msgs/PointCloud2` | 占据栅格转换的3D点云 |
| `/robot_plane_info` | - | 平面信息 |

## 快速开始

### 1. 环境依赖

```bash
# ROS Noetic
sudo apt-get install ros-noetic-desktop-full

# 依赖包
sudo apt-get install tmux \
                       ros-noetic-pcl-ros \
                       ros-noetic-visualization-msgs \
                       ros-noetic-nav-msgs
```

### 2. 编译工作空间

```bash
# 编译 Car 工作空间
cd Car
catkin_make
source devel/setup.bash

# 编译 Drone 工作空间
cd ../Drone
catkin_make
source devel/setup.bash
```

### 3. 一键启动

项目使用 tmux 创建 7 分屏布局同时运行所有组件：

```bash
./launch_all.sh
```

**终端布局：**
```
┌─────────┬─────────┬─────────┐
│ [1] Car  │ [2]Drone│ [3]Drone│
│ego_plann │ fitplane│ fit_pln │
├─────────┼─────────┼─────────┤
│ [4]Car_V │ [5]Dr_V │ [6]Py_ac│
│ Vis      │ Vis     │ acc_pt  │
├─────────┴─────────┴─────────┤
│         [7] Python          │
│       (transform)           │
└─────────────────────────────┘
```

### 4. tmux 操作

| 操作 | 命令 |
|------|------|
| 重新连接会话 | `tmux attach -t ros_all` |
| 关闭所有窗口 | `tmux kill-session -t ros_all` |
| 或使用脚本 | `./stop_all.sh` |

## 配置参数

### 规划器参数 (fitplane_planner)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `map_resolution` | 0.3 | 地图分辨率 (米) |
| `map_width_meters` | 100.0 | 地图宽度 (米) |
| `inflation_radius` | 1.5 | 障碍物膨胀半径 |
| `replanning_distance_threshold` | 0.5 | 重规划距离阈值 |
| `obstacle_distance_cost_weight` | 1.8 | 障碍物距离权重 |
| `turn_penalty_cost_weight` | 0.8 | 转向惩罚权重 |

### EGO-Planner 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `max_vel` | 2.0 | 最大速度 (m/s) |
| `max_acc` | 1.0 | 最大加速度 (m/s²) |
| `planning_horizon` | 2 | 规划时域 (秒) |
| `flight_type` | 3 | 飞行模式 (3=GRP模式) |

2026.04.01
对文件目录进行了修改，无编译报错
