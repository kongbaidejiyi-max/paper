#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from fitplane.msg import GridPlaneInfo
import matplotlib
matplotlib.use('Qt5Agg')

class HybridGridVisualizer:
    def __init__(self):
        rospy.init_node('hybrid_grid_visualizer', anonymous=True)
        
        # 创建简单的2x2布局，增大图像尺寸
        self.fig, self.axes = plt.subplots(2, 2, figsize=(14, 12))
        self.fig.suptitle('Grid Plane Analysis - Hybrid View', fontsize=16)
        
        # 设置子图标题
        titles = ['Traversability', 'Surface Angle', 'Height', 'Risk Map']
        for i, (ax, title) in enumerate(zip(self.axes.flat, titles)):
            ax.set_title(title, fontsize=12)
            ax.set_aspect('equal')
            ax.grid(True, alpha=0.3)  # 添加网格
        
        # 存储图像对象
        self.images = [None, None, None, None]
        self.scatters = [None, None, None, None]  # 存储散点图对象
        self.colorbars = [None, None, None, None]
        
        # 订阅话题
        rospy.Subscriber('grid_plane_info', GridPlaneInfo, self.callback_hybrid)
        
        plt.tight_layout()
        rospy.loginfo("Hybrid Grid Visualizer initialized")
    
    def callback_hybrid(self, msg):
        """混合显示：imshow + 散点图突出显示"""
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
            valid_mask = ~unknown_mask
            
            trav[unknown_mask] = np.nan
            angle[unknown_mask] = np.nan
            height_data[unknown_mask] = np.nan
            
            # 计算风险图
            risk = np.full_like(trav, np.nan)
            if np.any(valid_mask):
                risk[valid_mask] = (1 - trav[valid_mask]) * 0.7 + np.clip(angle[valid_mask]/30, 0, 1) * 0.3
            
            # 计算显示范围
            extent = [msg.origin.x, msg.origin.x + width * msg.resolution,
                     msg.origin.y, msg.origin.y + height * msg.resolution]
            
            # 数据和颜色映射
            data_list = [trav, angle, height_data, risk]
            cmaps = ['RdYlGn', 'coolwarm', 'terrain', 'RdYlGn_r']
            vlims = [(0, 1), (0, 30), (None, None), (0, 1)]
            
            # 准备散点图数据
            scatter_data = []
            if np.any(valid_mask):
                y_indices, x_indices = np.where(valid_mask)
                x_coords = msg.origin.x + x_indices * msg.resolution
                y_coords = msg.origin.y + y_indices * msg.resolution
                
                valid_trav = trav[valid_mask]
                valid_angle = angle[valid_mask]
                valid_height = height_data[valid_mask]
                valid_risk = risk[valid_mask]
                
                scatter_data = [valid_trav, valid_angle, valid_height, valid_risk]
            
            # 更新每个子图
            for i, (ax, data, cmap, vlim) in enumerate(zip(self.axes.flat, data_list, cmaps, vlims)):
                # 计算颜色范围
                if vlim[0] is None:
                    valid_data = data[~np.isnan(data)]
                    if len(valid_data) > 0:
                        vmin, vmax = np.min(valid_data), np.max(valid_data)
                    else:
                        vmin, vmax = 0, 1
                else:
                    vmin, vmax = vlim
                
                # 1. 先用imshow显示背景网格
                if self.images[i] is None:
                    self.images[i] = ax.imshow(data, origin='lower', extent=extent, 
                                             cmap=cmap, vmin=vmin, vmax=vmax,
                                             interpolation='nearest', alpha=0.6)  # 降低透明度
                    
                    # 创建颜色条
                    if self.colorbars[i] is not None:
                        self.colorbars[i].remove()
                    self.colorbars[i] = plt.colorbar(self.images[i], ax=ax, shrink=0.8)
                else:
                    self.images[i].set_data(data)
                    self.images[i].set_clim(vmin, vmax)
                
                # 2. 移除旧的散点图
                if self.scatters[i] is not None:
                    self.scatters[i].remove()
                    self.scatters[i] = None
                
                # 3. 添加散点图突出显示有效数据点
                if scatter_data and len(scatter_data[i]) > 0:
                    self.scatters[i] = ax.scatter(x_coords, y_coords, 
                                                c=scatter_data[i], 
                                                cmap=cmap, 
                                                s=0,  # 点的大小
                                                vmin=vmin, vmax=vmax, 
                                                alpha=0.9,  # 高透明度突出显示
                                                edgecolors='black',  # 黑色边框
                                                linewidth=0.3)  # 边框宽度
                
                # 设置显示范围
                ax.set_xlim(extent[0], extent[1])
                ax.set_ylim(extent[2], extent[3])
                
                # 添加一些统计信息到标题
                if scatter_data and len(scatter_data[i]) > 0:
                    mean_val = np.nanmean(scatter_data[i])
                    count = len(scatter_data[i])
                    titles = ['Traversability', 'Surface Angle', 'Height', 'Risk Map']
                    ax.set_title(f'{titles[i]}\n(Points: {count}, Mean: {mean_val:.2f})', fontsize=10)
            
            # 更新显示
            self.fig.canvas.draw_idle()
            
            rospy.logdebug(f"Updated visualization with {np.sum(valid_mask)} valid points")
            
        except Exception as e:
            rospy.logerr(f"Visualization error: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())
    
    def run(self):
        """运行可视化器"""
        try:
            plt.show()
            rospy.spin()  # 保持节点运行
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down visualizer")
        finally:
            plt.close('all')

if __name__ == '__main__':
    try:
        visualizer = HybridGridVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupted")
    except Exception as e:
        rospy.logerr(f"Error: {e}")
        import traceback
        rospy.logerr(traceback.format_exc())
