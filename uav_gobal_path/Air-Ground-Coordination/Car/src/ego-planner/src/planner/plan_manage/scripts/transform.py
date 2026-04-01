#!/usr/bin/env python3
import rospy
import numpy as np
import sensor_msgs.msg
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.point_cloud2 import create_cloud_xyz32
import std_msgs.msg

point_cloud_pub = None  # 全局发布者句柄


def occ_map_callback(msg: OccupancyGrid):
    """
    将 OccupancyGrid 中的占据栅格拉伸成沿 z 轴的柱状点云
    """
    # OccupancyGrid 一维数组 -> 二维
    data = np.array(msg.data, dtype=np.int8).reshape(
        (msg.info.height, msg.info.width)
    )

    origin_x = msg.info.origin.position.x
    origin_y = msg.info.origin.position.y
    resolution = msg.info.resolution
    width = msg.info.width
    height = msg.info.height

    # 只关心占据格子：值 == 100；忽略 -1（未知）
    occupied_mask = (data == 100)
    if not np.any(occupied_mask):
        rospy.logdebug("No occupied cells in map, skip publishing point cloud.")
        return

    # 找到占据格子的索引（row=i, col=j）
    occ_indices = np.argwhere(occupied_mask)
    is_ = occ_indices[:, 0]  # 行，对应 y
    js_ = occ_indices[:, 1]  # 列，对应 x

    # 计算每个占据栅格中心的 (x, y) 坐标
    xs = origin_x + (js_.astype(np.float32) + 0.5) * resolution
    ys = origin_y + (is_.astype(np.float32) + 0.5) * resolution

    # 拉伸高度和 z 方向间隔，可以在 launch 里用 rosparam 改
    max_height = rospy.get_param("~max_height", 2.0)   # 总高度，单位 m
    z_step = rospy.get_param("~z_step", resolution)    # z 方向采样间隔，默认跟地图分辨率一样

    # 生成所有 z 层
    z_levels = np.arange(0.0, max_height + 1e-6, z_step, dtype=np.float32)
    num_cells = xs.shape[0]
    num_levels = z_levels.shape[0]

    # 为每个栅格复制所有 z 层（向量化，不用 Python 双重 for）
    xs_rep = np.repeat(xs, num_levels)          # [x1,x1,...,x2,x2,...]
    ys_rep = np.repeat(ys, num_levels)
    zs_rep = np.tile(z_levels, num_cells)       # [z1,z2,...,zN,z1,z2,...]

    points = np.vstack((xs_rep, ys_rep, zs_rep)).T.astype(np.float32)

    if points.shape[0] == 0:
        rospy.logdebug("No points generated, skip publishing.")
        return

    # 创建 PointCloud2 消息
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "world"  # 如需跟机器人坐标系对齐，可改成 odom/base_link 等

    cloud = create_cloud_xyz32(header, points)
    point_cloud_pub.publish(cloud)
    rospy.logdebug("Published stretched point cloud with {} points.".format(points.shape[0]))


def listener():
    global point_cloud_pub

    rospy.init_node('occupancy_to_pointcloud', anonymous=True)

    # 点云发布者
    point_cloud_pub = rospy.Publisher(
        "/occupancy_point_cloud",
        sensor_msgs.msg.PointCloud2,
        queue_size=1
    )

    # 订阅 OccupancyGrid
    rospy.Subscriber("/fused_map", OccupancyGrid, occ_map_callback)

    rospy.loginfo("occupancy_to_pointcloud (stretched in z) node started.")
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
