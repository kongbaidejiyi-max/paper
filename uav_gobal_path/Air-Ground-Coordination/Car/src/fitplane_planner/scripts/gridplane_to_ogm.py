#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import OccupancyGrid
from fitplane.msg import GridPlaneInfo   # 确保包名/消息名和你工程里一致

class GridPlaneToOGM(object):
    def __init__(self):
        # 参数：话题名可以改
        fused_map_topic   = rospy.get_param("~fused_map_topic", "/fused_map")
        ogm_topic         = rospy.get_param("~ogm_topic", "/fused_map_occupancy")

        self.sub = rospy.Subscriber(fused_map_topic,
                                    GridPlaneInfo,
                                    self.cb_fused_map,
                                    queue_size=1)
        self.pub = rospy.Publisher(ogm_topic,
                                   OccupancyGrid,
                                   queue_size=1)

        rospy.loginfo("gridplane_to_ogm: subscribe %s, publish %s",
                      fused_map_topic, ogm_topic)

    def cb_fused_map(self, msg):
        if self.pub.get_num_connections() == 0:
            return

        ogm = OccupancyGrid()
        ogm.header = msg.header

        ogm.info.resolution = msg.resolution
        # 打印resolution
        # rospy.loginfo("resolution: %f", ogm.info.resolution)
        ogm.info.width      = msg.width
        ogm.info.height     = msg.height

        # origin: 位置来自你的 fused_map.origin，姿态设成单位四元数
        ogm.info.origin.position = msg.origin
        ogm.info.origin.orientation.x = 0.0
        ogm.info.origin.orientation.y = 0.0
        ogm.info.origin.orientation.z = 0.0
        ogm.info.origin.orientation.w = 1.0

        size = msg.width * msg.height
        ogm.data = [0] * size

        # occupancy: -1 未知, 0 空闲, 100 占用
        # nav_msgs/OccupancyGrid 也是用 [-1, 100] 的 int8，所以直接拷就行
        for i in range(size):
            occ = msg.occupancy[i]
            if occ < -1:
                occ = -1
            if occ > 100:
                occ = 100
            ogm.data[i] = int(occ)

        self.pub.publish(ogm)

def main():
    rospy.init_node("gridplane_to_ogm")
    node = GridPlaneToOGM()
    rospy.spin()

if __name__ == "__main__":
    main()
