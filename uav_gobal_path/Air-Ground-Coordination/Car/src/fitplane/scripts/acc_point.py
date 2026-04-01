#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from collections import deque
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import copy

class PointCloudAccumulator:
  def __init__(self):
      # 初始化ROS节点
      rospy.init_node('pointcloud_accumulator', anonymous=True)
      
      # 参数设置
      self.window_size = 5  # 滑动窗口大小
      self.input_topic = "/car/cloud_registered"
      self.output_topic = "/cloud_accumulateds"
      
      # 使用双端队列存储点云数据
      self.pointcloud_queue = deque(maxlen=self.window_size)
      
      # 存储原始点云的字段信息
      self.original_fields = None
      self.field_names = None
      self.original_frame_id = None
      
      # 统计信息
      self.total_received = 0
      self.total_published = 0
      self.last_publish_time = rospy.Time.now()
      
      # 发布器和订阅器
      self.subscriber = rospy.Subscriber(
          self.input_topic, 
          PointCloud2, 
          self.pointcloud_callback,
          queue_size=1
      )
      
      self.publisher = rospy.Publisher(
          self.output_topic, 
          PointCloud2, 
          queue_size=1
      )
      
      rospy.loginfo(f"点云累积器已启动")
      rospy.loginfo(f"输入话题: {self.input_topic}")
      rospy.loginfo(f"输出话题: {self.output_topic}")
      rospy.loginfo(f"滑动窗口大小: {self.window_size}")
  
  def analyze_pointcloud_fields(self, msg):
      """分析点云字段结构"""
      if self.original_fields is None:
          self.original_fields = msg.fields
          self.field_names = [field.name for field in msg.fields]
          self.original_frame_id = msg.header.frame_id
          
          rospy.loginfo(f"检测到点云坐标系: {self.original_frame_id}")
          rospy.loginfo("检测到点云字段:")
          for i, field in enumerate(msg.fields):
              rospy.loginfo(f"  {i}: {field.name} (type: {field.datatype}, offset: {field.offset})")
  
  def pointcloud_callback(self, msg):
      """点云回调函数"""
      try:
          self.total_received += 1
          
          # 检查输入点云是否为空
          if msg.width == 0 or msg.height == 0 or len(msg.data) == 0:
              rospy.logwarn(f"接收到空点云，跳过 (width: {msg.width}, height: {msg.height}, data_len: {len(msg.data)})")
              return
          
          # 分析字段结构（仅第一次）
          self.analyze_pointcloud_fields(msg)
          
          # 验证点云数据
          point_count = self.count_valid_points(msg)
          if point_count == 0:
              rospy.logwarn("接收到的点云没有有效点，跳过")
              return
          
          rospy.logdebug(f"接收到点云，包含 {point_count} 个有效点")
          
          # 直接将点云添加到队列（不进行坐标变换）
          processed_msg = copy.deepcopy(msg)
          processed_msg.header.stamp = rospy.Time.now()  # 更新时间戳
          
          self.pointcloud_queue.append(processed_msg)
          
          # 累积并发布点云
          self.accumulate_and_publish()
          
      except Exception as e:
          rospy.logerr(f"处理点云时出错: {str(e)}")
          import traceback
          rospy.logerr(f"详细错误信息: {traceback.format_exc()}")
  
  def count_valid_points(self, msg):
      """统计有效点的数量（采样方式，避免耗时过长）"""
      try:
          count = 0
          sample_limit = 1000  # 采样限制，避免计数时间过长
          for point in pc2.read_points(msg, skip_nans=True):
              count += 1
              if count >= sample_limit:
                  # 如果采样点数达到限制，估算总数
                  total_points = msg.width * msg.height
                  estimated_count = int(count * total_points / sample_limit)
                  rospy.logdebug(f"采样估算: {estimated_count} 个点")
                  return estimated_count
          return count
      except Exception as e:
          rospy.logwarn(f"统计点数失败: {str(e)}，使用消息尺寸估算")
          return msg.width * msg.height
  
  def accumulate_and_publish(self):
      """累积点云并发布"""
      if len(self.pointcloud_queue) == 0:
          rospy.logwarn("队列为空，无法发布")
          return
      
      try:
          # 收集所有点云数据
          all_points = []
          valid_clouds = 0
          total_points_per_cloud = []
          
          for i, cloud_msg in enumerate(self.pointcloud_queue):
              cloud_points = []
              try:
                  # 从每个点云消息中提取点
                  for point in pc2.read_points(cloud_msg, skip_nans=True):
                      cloud_points.append(point)
                  
                  if len(cloud_points) > 0:
                      all_points.extend(cloud_points)
                      valid_clouds += 1
                      total_points_per_cloud.append(len(cloud_points))
                  else:
                      rospy.logwarn(f"队列中第 {i} 个点云为空")
                      total_points_per_cloud.append(0)
                      
              except Exception as e:
                  rospy.logwarn(f"读取队列中第 {i} 个点云失败: {str(e)}")
                  total_points_per_cloud.append(0)
                  continue
          
          if len(all_points) == 0:
              rospy.logwarn(f"累积后没有有效点云数据，队列大小: {len(self.pointcloud_queue)}")
              rospy.logwarn(f"各帧点数: {total_points_per_cloud}")
              return
          
          # 创建累积后的点云消息
          accumulated_msg = self.create_pointcloud_message(all_points)
          
          if accumulated_msg is None:
              rospy.logerr("创建累积点云消息失败")
              return
          
          # 发布累积的点云
          self.publisher.publish(accumulated_msg)
          self.total_published += 1
          self.last_publish_time = rospy.Time.now()
          
          rospy.loginfo(f"发布累积点云: {len(all_points)} 点，来自 {valid_clouds}/{len(self.pointcloud_queue)} 帧")
          rospy.logdebug(f"各帧点数: {total_points_per_cloud}")
          rospy.logdebug(f"总接收: {self.total_received}, 总发布: {self.total_published}")
          
      except Exception as e:
          rospy.logerr(f"累积点云时出错: {str(e)}")
          import traceback
          rospy.logerr(f"详细错误信息: {traceback.format_exc()}")
  
  def create_pointcloud_message(self, points):
      """创建点云消息"""
      if len(points) == 0:
          rospy.logwarn("尝试创建空点云消息")
          return None
          
      # 创建消息头
      header = Header()
      header.stamp = rospy.Time.now()
      header.frame_id = self.original_frame_id if self.original_frame_id else "map"
      
      try:
          # 使用原始字段定义
          if self.original_fields is not None:
              pointcloud_msg = pc2.create_cloud(header, self.original_fields, points)
          else:
              # 备用字段定义
              fields = [
                  PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
              ]
              
              # 检查是否有额外字段
              if len(points) > 0 and len(points[0]) > 3:
                  fields.append(PointField('intensity', 12, PointField.FLOAT32, 1))
              
              pointcloud_msg = pc2.create_cloud(header, fields, points)
          
          return pointcloud_msg
          
      except Exception as e:
          rospy.logerr(f"创建点云消息失败: {str(e)}")
          return None
  
  def print_statistics(self):
      """打印统计信息"""
      rospy.loginfo(f"=== 点云累积器统计 ===")
      rospy.loginfo(f"接收帧数: {self.total_received}")
      rospy.loginfo(f"发布帧数: {self.total_published}")
      rospy.loginfo(f"当前队列大小: {len(self.pointcloud_queue)}")
      rospy.loginfo(f"坐标系: {self.original_frame_id}")
      if self.field_names:
          rospy.loginfo(f"字段: {', '.join(self.field_names)}")
  
  def run(self):
      """运行主循环"""
      rospy.loginfo("点云累积器开始运行...")
      
      # 定时打印统计信息
      def print_stats_callback(event):
          self.print_statistics()
      
      # 每30秒打印一次统计
      timer = rospy.Timer(rospy.Duration(30.0), print_stats_callback)
      
      try:
          rospy.spin()
      except KeyboardInterrupt:
          rospy.loginfo("接收到中断信号，正在关闭...")
      finally:
          timer.shutdown()
          self.print_statistics()  # 关闭前打印最终统计
          rospy.loginfo("点云累积器已关闭")

def main():
  try:
      accumulator = PointCloudAccumulator()
      accumulator.run()
  except rospy.ROSInterruptException:
      rospy.loginfo("ROS中断异常")
  except Exception as e:
      rospy.logerr(f"程序运行出错: {str(e)}")

if __name__ == '__main__':
  main()