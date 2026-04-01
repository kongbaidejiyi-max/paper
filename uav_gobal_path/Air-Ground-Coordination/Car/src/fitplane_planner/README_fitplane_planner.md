# Fitplane Planner: 智能无人机协同地形路径规划系统

## 1. 系统概述

`fitplane_planner` 是一个 ROS（Noetic）路径规划包，用于地面机器人在**实时更新的局部占据栅格**环境下执行导航任务。系统可接入无人机（或任意来源）发布的局部占据地图，通过 `MapManager` 融合成一张固定大小的全局栅格（锚定原点不滑窗），再由 `PathPlanner` 在该栅格上进行 A* 规划与探索式代理目标生成，并发布路径与走廊/置信度可视化。

本包只包含 **MapManager / PathPlanner** 两个节点；路径跟随（控制）不在本包内。

## 2. 核心功能与特色

### 2.1 动态地图管理 (`MapManager`)
- **固定大小全局栅格（锚定原点）**：首次收到机器人里程计后，将机器人置于地图中心并固定 `map_origin`，后续不滑窗，不清空历史，地图大小由 `map_width_meters / map_height_meters / map_resolution` 决定。
- **实时地图融合/拼图**：订阅局部 `nav_msgs/OccupancyGrid`（默认话题 `plane_OccMap`），将 `Free(0)/Occupied(100)` 覆盖写入全局栅格；**不会用 Unknown(-1) 擦除已知区域**。
- **时间戳防乱序**：对每个栅格记录 `last_updated`，旧时间戳的数据不会覆盖新数据。

### 2.2 智能探索式规划 (`PathPlanner`)
- **双模式规划**:
    - **标准模式**: 当最终目标点位于已知地图范围内时，执行高效的路径规划。
    - **探索模式**: 当最终目标点位于未知区域时，系统会自动切换到探索模式。
- **加权探索策略**: 在探索模式下，系统会综合考虑**最终目标的方向**和**无人机的当前位置**这两个因素。通过可配置的权重，它能计算出一个最佳的"代理目标点"，该点位于已知世界的边缘，能完美平衡"朝大目标前进"和"跟随无人机开拓新区域"这两种策略。
- **智能故障恢复**:
    - **无效目标修正**: 当用户不慎将目标点设置在障碍物内部时，系统会自动在目标点周围进行随机采样搜索，寻找一个附近的有效可达点作为新目标。
    - **不可达区域恢复**: 当A*算法因为目标被完全包围（如在"孤岛"区域）而找不到路径时，系统同样会触发随机搜索，尝试从该困境中"跳出"，寻找可替代的路径点。

### 2.3 高质量轨迹生成
- **代价感知 A\***：代价函数除基础通行性外，还会额外考虑：
    - **离障成本**：利用 `distance_to_obstacle_` 鼓励路径远离障碍物。
    - **拐弯惩罚**：惩罚频繁转向，让路径更平直。
    - **参考路径偏置（可选）**：`ref_path_bias_enable/ref_path_bias_weight` 可鼓励新路径靠近上一条已发布路径，减少抖动。
- **路径保持机制**: 为解决高频重规划导致的路径抖动和机器人"往返"移动问题，系统引入了"路径粘滞"逻辑。只有当机器人偏离当前路径超过一定阈值，或当前路径被新障碍物阻挡时，才会触发重规划，极大地提升了轨迹的稳定性和机器人运动的平顺性。
- **路径后处理**：对 A* 原始路径进行去环、（可选）重采样与适度简化，输出更稳定的离散路径点序列。

### 2.4 可视化支持
- **路径与走廊可视化**：
    - 路径：`/plan`（`nav_msgs/Path`）与 `/plan_marker`（Marker）。
    - 目标：`/goal_marker`（Marker）。
    - 走廊与置信度：`/corridor_marker`（Marker）与 `/grp`（`fitplane_planner/GlobalPath`，包含走廊半宽与置信度数组）。
- **性能注意**：可视化构建开销与点数相关；当 RViz 未订阅相关 marker 时，发布侧会尽量跳过不必要的可视化生成。

## 3. 节点与话题

### 3.1 节点
- **`map_manager_node`**: 地图管理器。负责维护和发布主地图。
- **`path_planner_node`**: 路径规划器。负责路径的计算、探索决策和故障恢复。

### 3.2 重要话题
- **输入**:
    - `plane_OccMap`：局部占据栅格（`nav_msgs/OccupancyGrid`，由无人机/感知端发布，供 `MapManager` 拼图）。
    - `odom`：地面机器人里程计（`nav_msgs/Odometry`，通常在 launch 里 remap 到 `/Odom_high_freq` 或仿真里程计）。
    - `drone/odom`：无人机里程计（`nav_msgs/Odometry`，用于探索代理目标加权；可在 launch 中 remap）。
    - `move_base_simple/goal`: RViz中设置的最终目标点。
- **输出/中间话题**:
    - `/fused_map`: 由`MapManager`发布的、融合后的主地图。`PathPlanner`从此获取地图信息。
    - `/plan`: `PathPlanner`发布的 A* 路径（`nav_msgs/Path`，供 `fitplane_trajectory_follower` 默认订阅）。
    - `/inflated_costmap`: `PathPlanner`发布的膨胀代价地图（可视化/调试）。
    - `/grp`: `fitplane_planner/GlobalPath`（路径点 + 走廊半宽 + 置信度）。
    - `/plan_marker`, `/goal_marker`, `/corridor_marker`: RViz 可视化 Marker。

## 4. 代码结构（开发者）

为了便于维护，`PathPlanner` 已拆分为 6 个模块（同一个类，功能不变）：

- `src/path_planner_runtime.cpp`：参数/订阅发布/主循环与探索逻辑
- `src/path_planner_map.cpp`：栅格稳定化、膨胀、距离场
- `src/path_planner_astar.cpp`：A* 搜索
- `src/path_planner_corridor.cpp`：走廊/安全性相关几何与检查
- `src/path_planner_resample.cpp`：重采样/局部重规划相关
- `src/path_planner_publish.cpp`：/plan、/grp、markers 的发布与可视化

## 5. 如何配置与运行

1.  **配置参数**: 主要的配置文件是 `launch/planner.launch`。您可以根据需求，调整其中的各项参数，尤其是**权重参数**，以改变规划器的"性格"。
    - `replanning_distance_threshold`: 机器人偏离多远才重规划，影响路径稳定性。
    - `rolling_replan_enable`: 是否在复用旧路径时“滚动裁剪”已走过前缀，让 `/plan` 起点跟随机器人前进（默认 true）。
    - `rolling_replan_keep_points_behind`: 裁剪时保留离机器人最近点之前的若干点（点数，默认 0；增大可减小抖动但可能让起点略落后）。
    - `stitch_replan_enable`: 是否启用“短段拼接式重规划”（默认 false；推荐在车行驶时打开）。
    - `stitch_replan_trigger_distance`: 小车每前进超过该距离（米）触发一次“短段重规划”。
    - `stitch_replan_lookahead_distance`: 从旧路径最近点开始，向前取该距离（米）的点作为拼接目标；只重规划到该点，然后复用旧路径剩余部分（减少抖动/跳变）。
    - `astar_raw_marker_enable`: 是否发布 A* 原始每步格子点的 Marker（topic: `astar_raw_marker`）。
    - `grp_centerline_marker_enable`: 是否发布 /grp 中心线点列的 Marker（topic: `grp_centerline_marker`）。
    - `obstacle_distance_cost_weight`: 路径有多"害怕"障碍物。
    - `turn_penalty_cost_weight`: 路径有多"喜欢"走直线。
    - `goal_weight`, `drone_pos_weight`: 在探索模式下，机器人更听"最终目标"的，还是更听"无人机"的。
    - `corridor_*`, `conf_w_*`, `occ_confirm_frames/free_confirm_frames`: 走廊与置信度/稳定栅格相关。
    - `corridor_viz_*`: `/corridor_marker` 走廊 3D 可视化外观（高度/透明度/边界线/内部网格/虚线/颜色模式等）。
    - `grp_resample_*`, `safe_resample_*`: GRP 发布与智能重采样相关。

2.  **配置话题**: 在 `launch/planner.launch` 文件中，确保将 `odom` 和 `drone/odom` 的 `remap` 指令，正确地指向您系统中实际发布机器人和无人机里程计的话题。
    - `PathPlanner` 的话题名均使用相对名（可直接 remap）：`fused_map`、`move_base_simple/goal`、`odom`、`drone/odom`、`plan`、`plan_marker`、`goal_marker`、`grp`、`inflated_costmap`、`corridor_marker`。
    - 其中 `odom` 仍支持用 rosparam 指定订阅名：`~odom_topic`（默认 `odom`）。

3.  **运行**:
    ```bash
    roslaunch fitplane_planner planner.launch
    ```
    同时确保局部占据栅格 `plane_OccMap` 与机器人里程计已在系统中发布。

## 6. 性能与调试（Timing）

规划器内置了分段耗时日志，终端会输出类似：

- `Timing(ms): total=... oldprep=... astar_full=... resample=... publish=...`
- `Timing(ms): gridInfoCallback total=... stable=... inflate=... pub_costmap=... plan=...`
- `Timing(ms): inflateCostmap total=... inflate=... dist_field=...`
- `Timing(ms): astar=... expanded=... pushed=... relax=... ...`
- `Timing(ms): grp total=... to_idx=... resample=... conf=... msg=... pub=... viz=...`

建议用下面命令快速筛选：

```bash
roslaunch fitplane_planner planner.launch 2>&1 | grep "Timing(ms):"
```
