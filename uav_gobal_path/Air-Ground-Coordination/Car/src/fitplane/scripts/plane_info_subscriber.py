#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import matplotlib.pyplot as plt
from fitplane.msg import PlaneMap, Plane
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

class PlaneInfoSubscriber:
    def __init__(self):
        rospy.init_node('plane_info_subscriber', anonymous=True)
        self.fig, self.axes = plt.subplots(2, 1, figsize=(10, 10))
        self.angle_values = []
        self.height_values = []
        self.time_values = []
        self.start_time = rospy.Time.now().to_sec()

        self.marker_pub = rospy.Publisher('/plane_info_marker', Marker, queue_size=10)
        rospy.Subscriber('robot_plane_info', PlaneMap, self.plane_info_callback)

        self.latest_angle = 0.0
        self.latest_height = 0.0
        self.data_updated = False

        rospy.loginfo("Plane info subscriber started, waiting for messages...")

    def plane_info_callback(self, msg):
        if not msg.PlaneGridMap:
            rospy.logwarn("Received empty plane info message")
            return

        plane_info = msg.PlaneGridMap[0]

        angle_deg = plane_info.PlaneCellAngle   # 直接就是角度
        height = plane_info.PlaneCellHeight     # 单位为米

        self.latest_angle = angle_deg
        self.latest_height = height

        current_time = rospy.Time.now().to_sec() - self.start_time
        self.time_values.append(current_time)
        self.angle_values.append(angle_deg)
        self.height_values.append(height)

        max_points = 100
        if len(self.time_values) > max_points:
            self.time_values = self.time_values[-max_points:]
            self.angle_values = self.angle_values[-max_points:]
            self.height_values = self.height_values[-max_points:]

        rospy.loginfo("Received plane info: angle=%.2f deg, height=%.3f m", angle_deg, height)
        print("【调试】最近绘图数据：", self.angle_values[-5:], self.height_values[-5:])

        self.publish_marker(height, angle_deg)
        self.data_updated = True

    def update_plot(self):
        for ax in self.axes:
            ax.clear()
            ax.get_yaxis().get_major_formatter().set_useOffset(False)
            ax.ticklabel_format(style='plain', axis='y')

        self.axes[0].plot(self.time_values, self.angle_values, 'r-')
        self.axes[0].set_title('Plane Angle (deg)')
        self.axes[0].set_xlabel('Time (s)')
        self.axes[0].set_ylabel('Angle (deg)')
        self.axes[0].grid(True)

        self.axes[1].plot(self.time_values, self.height_values, 'b-')
        self.axes[1].set_title('Plane Height (m)')
        self.axes[1].set_xlabel('Time (s)')
        self.axes[1].set_ylabel('Height (m)')
        self.axes[1].grid(True)

        plt.tight_layout()
        plt.pause(0.01)

    def publish_marker(self, height, angle_deg):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "plane_info"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.z = height + 0.5

        marker.text = "Angle: %.2f°\nHeight: %.3fm" % (angle_deg, height)
        marker.scale.z = 0.2

        normalized_angle = min(1.0, angle_deg / 40.0)
        color = ColorRGBA()
        color.r = normalized_angle
        color.g = 1.0 - normalized_angle
        color.b = 0.0
        color.a = 1.0
        marker.color = color

        self.marker_pub.publish(marker)

    def run(self):
        plt.ion()
        rate = rospy.Rate(10)

        try:
            while not rospy.is_shutdown():
                if self.data_updated:
                    self.update_plot()
                    self.data_updated = False
                rate.sleep()
        except KeyboardInterrupt:
            rospy.loginfo("Subscriber interrupted by user")
        finally:
            plt.ioff()
            plt.close()

if __name__ == '__main__':
    try:
        subscriber = PlaneInfoSubscriber()
        subscriber.run()
    except rospy.ROSInterruptException:
        pass
