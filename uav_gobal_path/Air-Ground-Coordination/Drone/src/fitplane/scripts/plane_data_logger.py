#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import csv
import os
import numpy as np
from datetime import datetime
from fitplane.msg import PlaneMap, Plane
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped

class PlaneDataLogger:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('plane_data_logger', anonymous=True)
        
        # 创建输出目录
        self.output_dir = os.path.expanduser("~/plane_data")
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            
        # 创建CSV文件用于数据记录
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = os.path.join(self.output_dir, f"plane_data_{timestamp}.csv")
        self.csv_file = open(self.csv_filename, 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['时间戳', '角度(度)', '高度(米)'])
        
        # 创建统计数据发布器
        self.angle_pub = rospy.Publisher('/plane_angle', Float32, queue_size=10)
        self.height_pub = rospy.Publisher('/plane_height', Float32, queue_size=10)
        
        # 订阅平面信息话题
        rospy.Subscriber('robot_plane_info', PlaneMap, self.plane_info_callback)
        rospy.Subscriber('/robot_pose', PoseStamped, self.robot_pose_callback, queue_size=10)
        
        # 存储数据
        self.angle_values = []
        self.height_values = []
        self.robot_pose = None
        
        # 统计信息
        self.count = 0
        self.start_time = rospy.Time.now()
        
        rospy.loginfo("平面数据记录器已启动，数据将保存到: %s", self.csv_filename)
        
    def plane_info_callback(self, msg):
        """处理接收到的平面信息消息"""
        if not msg.PlaneGridMap:
            return
            
        # 获取平面信息
        plane_info = msg.PlaneGridMap[0]
        angle = plane_info.PlaneCellAngle
        height = plane_info.PlaneCellHeight
        
        # 记录时间戳
        timestamp = rospy.Time.now()
        elapsed_time = (timestamp - self.start_time).to_sec()
        
        # 存储数据
        self.angle_values.append(angle)
        self.height_values.append(height)
        
        # 写入CSV
        self.csv_writer.writerow([timestamp.to_sec(), angle, height])
        self.csv_file.flush()  # 确保数据立即写入文件
        
        # 更新统计信息
        self.count += 1
        if self.count % 10 == 0:  # 每10条消息打印一次统计信息
            self.print_statistics()
            
        # 发布统计数据
        angle_msg = Float32()
        angle_msg.data = angle
        self.angle_pub.publish(angle_msg)
        
        height_msg = Float32()
        height_msg.data = height
        self.height_pub.publish(height_msg)
        
        # 打印简洁信息
        rospy.loginfo("[%.1fs] 平面: 角度=%.2f°, 高度=%.3fm", elapsed_time, angle, height)
        
    def robot_pose_callback(self, msg):
        """处理机器人位姿信息"""
        self.robot_pose = msg.pose
        
    def print_statistics(self):
        """打印统计信息"""
        if not self.angle_values:
            return
            
        # 计算统计数据
        avg_angle = np.mean(self.angle_values)
        avg_height = np.mean(self.height_values)
        std_angle = np.std(self.angle_values)
        std_height = np.std(self.height_values)
        
        # 打印统计信息
        rospy.loginfo("=" * 50)
        rospy.loginfo("统计信息 (样本数: %d)", self.count)
        rospy.loginfo("角度: 平均=%.2f°, 标准差=%.2f°, 最小=%.2f°, 最大=%.2f°", 
                     avg_angle, std_angle, min(self.angle_values), max(self.angle_values))
        rospy.loginfo("高度: 平均=%.3fm, 标准差=%.3fm, 最小=%.3fm, 最大=%.3fm", 
                     avg_height, std_height, min(self.height_values), max(self.height_values))
        rospy.loginfo("=" * 50)
        
    def calculate_terrain_roughness(self):
        """计算地形粗糙度指标"""
        if len(self.angle_values) < 10:
            return 0.0
            
        # 使用角度变化的标准差作为粗糙度指标
        return np.std(self.angle_values[-10:])
        
    def close(self):
        """关闭文件并打印总结"""
        if self.csv_file:
            self.csv_file.close()
            
        if not self.angle_values:
            return
            
        # 打印最终统计信息
        rospy.loginfo("数据记录完成，共记录 %d 条数据点", self.count)
        rospy.loginfo("数据已保存到: %s", self.csv_filename)
        
    def run(self):
        """运行主循环"""
        rate = rospy.Rate(1)  # 1Hz的状态更新
        
        try:
            while not rospy.is_shutdown():
                # 计算并发布地形粗糙度
                roughness = self.calculate_terrain_roughness()
                rate.sleep()
                
        except KeyboardInterrupt:
            rospy.loginfo("数据记录器被用户中断")
        finally:
            self.close()
            
if __name__ == '__main__':
    try:
        logger = PlaneDataLogger()
        logger.run()
    except rospy.ROSInterruptException:
        pass 