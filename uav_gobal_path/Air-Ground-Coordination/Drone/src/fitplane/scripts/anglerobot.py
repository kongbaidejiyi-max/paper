#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import tf
import math
import time
import threading

# 导入matplotlib时处理可能的错误
try:
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    from collections import deque
    MATPLOTLIB_AVAILABLE = True
except ImportError as e:
    print(f"Matplotlib import error: {e}")
    MATPLOTLIB_AVAILABLE = False

class SlopeVisualizer:
    def __init__(self):
        self.slope_data = deque(maxlen=200)  # 增加数据点
        self.time_data = deque(maxlen=200)
        self.roll_data = deque(maxlen=200)   # 单独显示roll
        self.pitch_data = deque(maxlen=200)  # 单独显示pitch
        self.start_time = None
        self.fig = None
        self.ax = None
        
    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
        
        # 转换为度数
        roll_deg = roll * 180.0 / math.pi
        pitch_deg = pitch * 180.0 / math.pi
        slope_deg = math.sqrt(roll**2 + pitch**2) * 180.0 / math.pi
        
        if self.start_time is None:
            self.start_time = rospy.get_time()
        t = rospy.get_time() - self.start_time
        
        self.slope_data.append(slope_deg)
        self.roll_data.append(roll_deg)
        self.pitch_data.append(pitch_deg)
        self.time_data.append(t)
        
        # 在终端打印当前值以便调试
        if len(self.slope_data) % 10 == 0:  # 每10个数据点打印一次
            print(f"Roll: {roll_deg:.3f}°, Pitch: {pitch_deg:.3f}°, Slope: {slope_deg:.3f}°")
        
    def update_plot(self):
        if not MATPLOTLIB_AVAILABLE or len(self.time_data) == 0:
            return
            
        if self.fig is None:
            self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(12, 8))
            plt.ion()  # 开启交互模式
            
        # 清除两个子图
        self.ax1.clear()
        self.ax2.clear()
        
        # 第一个子图：Roll和Pitch分别显示
        self.ax1.plot(self.time_data, self.roll_data, 'b-', linewidth=2, label='Roll')
        self.ax1.plot(self.time_data, self.pitch_data, 'g-', linewidth=2, label='Pitch')
        self.ax1.set_ylabel('Angle (°)')
        self.ax1.set_title('Roll and Pitch Angles')
        self.ax1.legend()
        self.ax1.grid(True, alpha=0.3)
        
        # 动态调整y轴范围以显示微小变化
        if self.roll_data and self.pitch_data:
            all_angles = list(self.roll_data) + list(self.pitch_data)
            min_angle = min(all_angles)
            max_angle = max(all_angles)
            margin = max(0.1, (max_angle - min_angle) * 0.1)  # 至少0.1度的边距
            self.ax1.set_ylim([min_angle - margin, max_angle + margin])
        
        # 第二个子图：合成坡度
        self.ax2.plot(self.time_data, self.slope_data, 'r-', linewidth=2, label='Combined Slope')
        self.ax2.set_ylabel('Slope (°)')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_title('Combined Slope Visualization')
        self.ax2.legend()
        self.ax2.grid(True, alpha=0.3)
        
        # 动态调整坡度的y轴范围
        if self.slope_data:
            min_slope = min(self.slope_data)
            max_slope = max(self.slope_data)
            margin = max(0.1, (max_slope - min_slope) * 0.1)
            self.ax2.set_ylim([min_slope - margin, max_slope + margin])
        
        # 显示当前值
        if self.slope_data and self.roll_data and self.pitch_data:
            current_slope = self.slope_data[-1]
            current_roll = self.roll_data[-1]
            current_pitch = self.pitch_data[-1]
            
            # 在第一个子图显示当前Roll和Pitch
            self.ax1.text(0.02, 0.95, f'Roll: {current_roll:.3f}°', 
                        transform=self.ax1.transAxes, fontsize=10,
                        bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
            self.ax1.text(0.02, 0.85, f'Pitch: {current_pitch:.3f}°', 
                        transform=self.ax1.transAxes, fontsize=10,
                        bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8))
            
            # 在第二个子图显示当前坡度
            self.ax2.text(0.02, 0.95, f'Slope: {current_slope:.3f}°', 
                        transform=self.ax2.transAxes, fontsize=10,
                        bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.8))
        
        plt.tight_layout()
        plt.draw()
        plt.pause(0.01)

def main():
    visualizer = SlopeVisualizer()
    
    rospy.init_node('slope_viz')
    rospy.Subscriber('/Odom_high_freq', Odometry, visualizer.odom_callback)
    
    if MATPLOTLIB_AVAILABLE:
        print("开始可视化坡度数据...")
        print("数据格式: Roll, Pitch, Combined Slope (单位: 度)")
        rate = rospy.Rate(10)  # 10Hz更新频率
        
        while not rospy.is_shutdown():
            visualizer.update_plot()
            rate.sleep()
    else:
        print("Matplotlib不可用，只运行ROS节点...")
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("程序被用户中断")
    except Exception as e:
        print(f"程序错误: {e}")
