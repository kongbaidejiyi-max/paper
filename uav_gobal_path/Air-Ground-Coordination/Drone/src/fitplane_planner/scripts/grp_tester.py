#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from fitplane_planner.msg import GlobalPath
import time

class GRPTester:
    def __init__(self):
        """初始化GRP测试器"""
        self.message_count = 0

        # 初始化ROS节点
        rospy.init_node('grp_tester', anonymous=True)

        # 订阅 /grp 话题
        rospy.Subscriber('/grp', GlobalPath, self.grp_callback)

        rospy.loginfo("GRP Tester Node Started")
        rospy.loginfo("Subscribed to topic: /grp")
        rospy.loginfo("Waiting for GlobalPath messages...")

    def grp_callback(self, msg):
        """回调函数，处理接收到的GlobalPath消息"""
        self.message_count += 1

        print("\n=== Received GlobalPath Message #{} ===".format(self.message_count))
        print("Header:")
        print("  - Frame ID: {}".format(msg.header.frame_id))
        print("  - Stamp: {}".format(msg.header.stamp))

        print("\nPath Info:")
        print("  - Map Version: {}".format(msg.map_version))
        print("  - Number of Poses: {}".format(len(msg.poses)))

        if len(msg.poses) == 0:
            print("  - No path poses!")
        else:
            # 显示起点和终点
            start_pose = msg.poses[0]
            goal_pose = msg.poses[-1]
            print("  - Start Pose: [{:.3f}, {:.3f}, {:.3f}]".format(
                start_pose.position.x,
                start_pose.position.y,
                start_pose.position.z))
            print("  - Goal Pose: [{:.3f}, {:.3f}, {:.3f}]".format(
                goal_pose.position.x,
                goal_pose.position.y,
                goal_pose.position.z))

            print("\n  Path Poses ({} poses):".format(len(msg.poses)))
            # 显示前5个点和最后5个点
            show_count = min(5, len(msg.poses))
            print("  First {} poses:".format(show_count))
            for i in range(show_count):
                p = msg.poses[i].position
                print("    Pose {}: [{:.3f}, {:.3f}, {:.3f}]".format(i, p.x, p.y, p.z))

            if len(msg.poses) > 10:
                print("  ... {} more poses ...".format(len(msg.poses) - 10))
                print("  Last {} poses:".format(show_count))
                for i in range(len(msg.poses) - show_count, len(msg.poses)):
                    p = msg.poses[i].position
                    print("    Pose {}: [{:.3f}, {:.3f}, {:.3f}]".format(i, p.x, p.y, p.z))

            # 显示走廊宽度和置信度信息
            if len(msg.corridor_half_width) > 0:
                print("\n  Corridor Info:")
                widths_str = ', '.join('{:.3f}'.format(w) for w in msg.corridor_half_width[:5])
                print("    - First 5 corridor half widths: {}m".format(widths_str))
                if len(msg.corridor_half_width) > 5:
                    print("    ... and {} more".format(len(msg.corridor_half_width) - 5))

            if len(msg.path_confidence) > 0:
                confidences_str = ', '.join('{:.3f}'.format(c) for c in msg.path_confidence[:5])
                print("    - First 5 path confidences: {}".format(confidences_str))
                if len(msg.path_confidence) > 5:
                    print("    ... and {} more".format(len(msg.path_confidence) - 5))

        print("=" * 50)
        print()

    def run(self):
        """运行测试器"""
        rate = rospy.Rate(10)  # 10 Hz
        counter = 0

        while not rospy.is_shutdown():
            counter += 1

            # 每10秒打印一次状态
            if counter >= 100:
                rospy.loginfo("Still listening... (received %d messages so far)", self.message_count)
                counter = 0

            rate.sleep()

def main():
    try:
        tester = GRPTester()
        tester.run()
    except rospy.ROSInterruptException:
        print("\nNode interrupted by user")
    except Exception as e:
        rospy.logerr("Exception: %s", str(e))
        return -1

    return 0

if __name__ == '__main__':
    main()