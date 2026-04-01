#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from fitplane.msg import GridPlaneInfo
import matplotlib
matplotlib.use('Qt5Agg')

class SimpleRegionVisualizer:
    def __init__(self):
        rospy.init_node('simple_region_visualizer', anonymous=True)
        
        # 创建2x2布局
        self.fig, self.axes = plt.subplots(2, 2, figsize=(16, 12))
        self.fig.suptitle('Grid Plane Analysis - Block View', fontsize=16)
        
        # 设置子图标题
        titles = ['Traversability', 'Surface Angle (°)', 'Height (m)', 'Risk Level']
        for i, (ax, title) in enumerate(zip(self.axes.flat, titles)):
            ax.set_title(title, fontsize=12)
            ax.set_aspect('equal')
            ax.grid(True, alpha=0.3)
        
        # 存储图像对象
        self.meshes = [None, None, None, None]
        self.colorbars = [None, None, None, None]
        
        # 订阅话题
        rospy.Subscriber('grid_plane_info', GridPlaneInfo, self.callback_mesh)
        
        plt.tight_layout()
        rospy.loginfo("Simple Region Visualizer initialized")
    
    def callback_mesh(self, msg):
        """使用pcolormesh显示区域"""
        try:
            # 基本数据处理
            width, height = msg.width, msg.height
            if width <= 0 or height <= 0:
                return
                
            # 重塑数据
            trav = np.array(msg.traversability).reshape(height, width).astype(float)
            angle = np.array(msg.plane_angle).reshape(height, width).astype(float)
            height_data = np.array(msg.plane_height).reshape(height, width).astype(float)
            
            # 处理未知值
            unknown_mask = (trav == -1)
            trav[unknown_mask] = np.nan
            angle[unknown_mask] = np.nan
            height_data[unknown_mask] = np.nan
            
            # 计算风险图
            risk = np.full_like(trav, np.nan)
            valid_mask = ~unknown_mask
            if np.any(valid_mask):
                risk[valid_mask] = (1 - trav[valid_mask]) * 0.7 + np.clip(angle[valid_mask]/30, 0, 1) * 0.3
            
            # 创建网格坐标 (用于pcolormesh)
            x_edges = np.linspace(msg.origin.x, msg.origin.x + width * msg.resolution, width + 1)
            y_edges = np.linspace(msg.origin.y, msg.origin.y + height * msg.resolution, height + 1)
            X, Y = np.meshgrid(x_edges, y_edges)
            
            # 数据和颜色映射
            data_list = [trav, angle, height_data, risk]
            cmaps = ['RdYlGn', 'coolwarm', 'terrain', 'RdYlGn_r']
            vlims = [(0, 1), (0, 30), (None, None), (0, 1)]
            titles = ['Traversability', 'Surface Angle (°)', 'Height (m)', 'Risk Level']
            
            # 更新每个子图
            for i, (ax, data, cmap, vlim, title) in enumerate(zip(self.axes.flat, data_list, cmaps, vlims, titles)):
                
                # 计算颜色范围
                if vlim[0] is None:
                    valid_data = data[~np.isnan(data)]
                    if len(valid_data) > 0:
                        vmin, vmax = np.min(valid_data), np.max(valid_data)
                        if vmin == vmax:
                            vmax = vmin + 1
                    else:
                        vmin, vmax = 0, 1
                else:
                    vmin, vmax = vlim
                
                # 移除旧的mesh
                if self.meshes[i] is not None:
                    self.meshes[i].remove()
                
                # 创建新的pcolormesh (每个网格单元显示为一个区块)
                self.meshes[i] = ax.pcolormesh(X, Y, data, 
                                             cmap=cmap, 
                                             vmin=vmin, vmax=vmax,
                                             shading='flat',  # 每个单元格显示为平坦区块
                                             alpha=0.8,
                                             edgecolors='black',  # 添加黑色边框
                                             linewidth=0.1)  # 细边框
                
                # 更新颜色条
                if self.colorbars[i] is not None:
                    self.colorbars[i].remove()
                self.colorbars[i] = plt.colorbar(self.meshes[i], ax=ax, shrink=0.8)
                
                # 统计信息
                valid_data = data[~np.isnan(data)]
                if len(valid_data) > 0:
                    mean_val = np.mean(valid_data)
                    ax.set_title(f'{title}\nCells: {len(valid_data)}, Mean: {mean_val:.2f}', fontsize=10)
                else:
                    ax.set_title(f'{title}\nNo valid data', fontsize=10)
                
                # 设置坐标轴
                ax.set_xlabel('X (m)', fontsize=9)
                ax.set_ylabel('Y (m)', fontsize=9)
            
            # 更新显示
            self.fig.canvas.draw_idle()
            
        except Exception as e:
            rospy.logerr(f"Visualization error: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())
    
    def run(self):
        """运行可视化器"""
        try:
            plt.show()
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down visualizer")
        finally:
            plt.close('all')

if __name__ == '__main__':
    try:
        visualizer = SimpleRegionVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupted")
    except Exception as e:
        rospy.logerr(f"Error: {e}")
