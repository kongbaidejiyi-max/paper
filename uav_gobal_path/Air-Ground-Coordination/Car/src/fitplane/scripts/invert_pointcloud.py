#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

class PointCloudInverter:
    def __init__(self):
        rospy.init_node('pointcloud_inverter', anonymous=True)

        # 参数设置
        self.input_topic = rospy.get_param('~input_topic', '/cloud_accumulateds')
        self.output_topic = rospy.get_param('~output_topic', '/cloud_registered_inverted')

        # 统计
        self.count = 0

        # 订阅和发布
        self.sub = rospy.Subscriber(self.input_topic, PointCloud2, self.callback, queue_size=1)
        self.pub = rospy.Publisher(self.output_topic, PointCloud2, queue_size=1)

        rospy.loginfo("点云Z轴反转节点启动")
        rospy.loginfo(f"输入: {self.input_topic} -> 输出: {self.output_topic}")

    def callback(self, msg):
        try:
            # 获取所有字段名
            field_names = [f.name for f in msg.fields]

            # 读取并反转Z轴
            points = []
            for p in pc2.read_points(msg, field_names=field_names, skip_nans=False):
                p_list = list(p)
                # 找到z字段的索引
                if 'z' in field_names:
                    z_idx = field_names.index('z')
                    p_list[z_idx] = -p_list[z_idx]  # Z轴取反
                points.append(tuple(p_list))

            # 使用公开API创建新消息
            new_msg = pc2.create_cloud(msg.header, msg.fields, points)
            new_msg.header.stamp = rospy.Time.now()

            self.pub.publish(new_msg)

            self.count += 1
            if self.count % 10 == 0:
                rospy.loginfo(f"已处理 {self.count} 帧")

        except Exception as e:
            rospy.logerr(f"错误: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())

    def run(self):
        rospy.loginfo("开始运行...")
        rospy.spin()

if __name__ == '__main__':
    try:
        node = PointCloudInverter()
        node.run()
    except rospy.ROSInterruptException:
        pass
