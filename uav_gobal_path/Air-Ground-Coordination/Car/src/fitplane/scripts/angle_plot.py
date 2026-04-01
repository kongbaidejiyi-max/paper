#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from fitplane.msg import GridPlaneInfo
import matplotlib
matplotlib.use('Qt5Agg')

class TraversabilityAngleVisualizer:
    def __init__(self):
        rospy.init_node('traversability_angle_visualizer', anonymous=True)
        
        # 创建1x2布局，并排显示两个图
        self.fig, self.axes = plt.subplots(1, 2, figsize=(16, 8))
        self.fig.suptitle('Traversability and Surface Angle Analysis (Click to view values)', fontsize=16)
        
        # 设置两个子图
        titles = ['Traversability (Higher = Less Traversable)', 'Surface Angle (Higher = Steeper)']
        xlabels = ['X (m)', 'X (m)']
        ylabels = ['Y (m)', 'Y (m)']
        
        for i, (ax, title, xlabel, ylabel) in enumerate(zip(self.axes, titles, xlabels, ylabels)):
            ax.set_title(title, fontsize=12)
            ax.set_aspect('equal')
            ax.grid(True, alpha=0.3)
            ax.set_xlabel(xlabel, fontsize=10)
            ax.set_ylabel(ylabel, fontsize=10)
        
        # 存储图像对象和数据
        self.images = [None, None]
        self.colorbars = [None, None]
        self.current_data = [None, None]  # 存储当前数据
        self.current_extent = None
        self.current_resolution = None
        self.current_origin = None
        
        # 添加文本显示框
        self.info_text = self.fig.text(0.02, 0.98, '', transform=self.fig.transFigure, 
                                      fontsize=10, verticalalignment='top',
                                      bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        # 连接鼠标事件
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_hover)
        
        # 订阅话题
        rospy.Subscriber('grid_plane_info', GridPlaneInfo, self.callback_dual_display)
        
        plt.tight_layout()
        rospy.loginfo("Traversability and Angle Visualizer initialized")
        rospy.loginfo("Click on the plots to view exact values at that position")
    
    def world_to_grid(self, x, y):
        """将世界坐标转换为网格坐标"""
        if self.current_origin is None or self.current_resolution is None:
            return None, None
        
        grid_x = int((x - self.current_origin.x) / self.current_resolution)
        grid_y = int((y - self.current_origin.y) / self.current_resolution)
        
        return grid_x, grid_y
    
    def get_value_at_position(self, x, y, data_index):
        """获取指定位置的数值"""
        if self.current_data[data_index] is None:
            return None
        
        grid_x, grid_y = self.world_to_grid(x, y)
        if grid_x is None or grid_y is None:
            return None
        
        height, width = self.current_data[data_index].shape
        if 0 <= grid_x < width and 0 <= grid_y < height:
            return self.current_data[data_index][grid_y, grid_x]
        
        return None
    
    def on_click(self, event):
        """鼠标点击事件处理"""
        if event.inaxes is None:
            return
        
        # 确定点击的是哪个子图
        ax_index = None
        for i, ax in enumerate(self.axes):
            if event.inaxes == ax:
                ax_index = i
                break
        
        if ax_index is None:
            return
        
        x, y = event.xdata, event.ydata
        if x is None or y is None:
            return
        
        # 获取两个图的数值
        trav_val = self.get_value_at_position(x, y, 0)
        angle_val = self.get_value_at_position(x, y, 1)
        
        # 格式化显示信息
        info_lines = [
            f"Position: ({x:.2f}, {y:.2f}) m",
            f"Grid: {self.world_to_grid(x, y)}",
        ]
        
        if trav_val is not None and not np.isnan(trav_val):
            info_lines.append(f"Traversability: {trav_val:.4f}")
        else:
            info_lines.append("Traversability: No data")
        
        if angle_val is not None and not np.isnan(angle_val):
            info_lines.append(f"Surface Angle: {angle_val:.2f}°")
        else:
            info_lines.append("Surface Angle: No data")
        
        # 添加可通行性评估
        if trav_val is not None and not np.isnan(trav_val):
            if trav_val < 0.3:
                trav_desc = "Easy"
            elif trav_val < 0.7:
                trav_desc = "Moderate"
            else:
                trav_desc = "Difficult"
            info_lines.append(f"Assessment: {trav_desc}")
        
        # 添加角度评估
        if angle_val is not None and not np.isnan(angle_val):
            if angle_val < 15:
                angle_desc = "Flat"
            elif angle_val < 45:
                angle_desc = "Moderate slope"
            else:
                angle_desc = "Steep"
            info_lines.append(f"Slope: {angle_desc}")
        
        self.info_text.set_text('\n'.join(info_lines))
        self.fig.canvas.draw_idle()
        
        rospy.loginfo(f"Clicked at ({x:.2f}, {y:.2f}): Trav={trav_val:.4f if trav_val is not None and not np.isnan(trav_val) else 'N/A'}, "
                     f"Angle={angle_val:.2f if angle_val is not None and not np.isnan(angle_val) else 'N/A'}°")
    
    def on_hover(self, event):
        """鼠标悬停事件处理（可选）"""
        # 如果你想要悬停显示，可以取消注释下面的代码
        # 但这可能会让显示更新过于频繁
        pass
    
    def callback_dual_display(self, msg):
        """显示可通行性和角度地图"""
        try:
            # 基本数据处理
            width, height = msg.width, msg.height
            if width <= 0 or height <= 0:
                return
                
            # 重塑数据
            trav = np.array(msg.traversability).reshape(height, width).astype(float)
            angle = np.array(msg.plane_angle).reshape(height, width).astype(float)
            
            # 处理未知值
            unknown_mask = (trav == -1)
            trav[unknown_mask] = np.nan
            angle[unknown_mask] = np.nan
            
            # 存储当前数据用于交互
            self.current_data = [trav, angle]
            self.current_resolution = msg.resolution
            self.current_origin = msg.origin
            
            # 计算显示范围
            extent = [msg.origin.x, msg.origin.x + width * msg.resolution,
                     msg.origin.y, msg.origin.y + height * msg.resolution]
            self.current_extent = extent
            
            # 数据、颜色映射和范围
            data_list = [trav, angle]
            cmaps = ['RdYlGn_r', 'coolwarm']  # 可通行性用红绿，角度用冷暖色调
            vlims = [(0, 1), (0, 90)]  # 可通行性0-1，角度0-90度
            colorbar_labels = [
                'Traversability Value\n(0=Easy, 1=Difficult)',
                'Surface Angle (degrees)\n(0=Flat, 90=Vertical)'
            ]
            
            # 更新每个子图
            for i, (ax, data, cmap, vlim, cb_label) in enumerate(zip(self.axes, data_list, cmaps, vlims, colorbar_labels)):
                
                # 计算颜色范围
                valid_data = data[~np.isnan(data)]
                if len(valid_data) > 0:
                    if vlim[0] is not None:
                        vmin, vmax = vlim
                        # 对于角度，如果实际最大值小于90，可以用实际范围
                        if i == 1:  # 角度图
                            actual_max = np.max(valid_data)
                            if actual_max < 90:
                                vmax = 90  # 保持固定90度范围，便于比较
                    else:
                        vmin, vmax = np.min(valid_data), np.max(valid_data)
                        if vmin == vmax:
                            vmax = vmin + 1
                else:
                    vmin, vmax = vlim if vlim[0] is not None else (0, 1)
                
                # 创建或更新图像
                if self.images[i] is None:
                    self.images[i] = ax.imshow(data, 
                                             origin='lower', 
                                             extent=extent, 
                                             cmap=cmap,
                                             vmin=vmin, 
                                             vmax=vmax,
                                             interpolation='nearest',
                                             alpha=0.8)
                    
                    # 创建颜色条
                    if self.colorbars[i] is not None:
                        self.colorbars[i].remove()
                    self.colorbars[i] = plt.colorbar(self.images[i], ax=ax, shrink=0.8)
                    self.colorbars[i].set_label(cb_label, fontsize=9)
                    
                    # 为角度图添加特殊的刻度标记
                    if i == 1:  # 角度图
                        self.colorbars[i].set_ticks([0, 15, 30, 45, 60, 75, 90])
                        self.colorbars[i].set_ticklabels(['0°', '15°', '30°', '45°', '60°', '75°', '90°'])
                else:
                    # 更新现有图像
                    self.images[i].set_data(data)
                    self.images[i].set_clim(vmin, vmax)
                
                # 设置显示范围
                ax.set_xlim(extent[0], extent[1])
                ax.set_ylim(extent[2], extent[3])
                
                # 添加统计信息到标题
                if len(valid_data) > 0:
                    mean_val = np.mean(valid_data)
                    min_val = np.min(valid_data)
                    max_val = np.max(valid_data)
                    count = len(valid_data)
                    
                    if i == 0:  # 可通行性
                        title_text = (f'Traversability (Click to view values)\n'
                                    f'Cells: {count}, Mean: {mean_val:.3f}, '
                                    f'Range: [{min_val:.3f}, {max_val:.3f}]')
                    else:  # 角度
                        title_text = (f'Surface Angle (Click to view values)\n'
                                    f'Cells: {count}, Mean: {mean_val:.1f}°, '
                                    f'Range: [{min_val:.1f}°, {max_val:.1f}°]')
                        
                        # 添加角度分类信息
                        flat_count = np.sum(valid_data < 15)
                        moderate_count = np.sum((valid_data >= 15) & (valid_data < 45))
                        steep_count = np.sum(valid_data >= 45)
                        
                        title_text += f'\nFlat(<15°): {flat_count}, Moderate(15-45°): {moderate_count}, Steep(≥45°): {steep_count}'
                    
                    ax.set_title(title_text, fontsize=9)
                else:
                    titles = ['Traversability\nNo valid data', 'Surface Angle\nNo valid data']
                    ax.set_title(titles[i], fontsize=10)
            
            # 更新显示
            self.fig.canvas.draw_idle()
            
            rospy.logdebug(f"Updated dual visualization with {len(trav[~np.isnan(trav)])} valid cells")
            
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
        visualizer = TraversabilityAngleVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupted")
    except Exception as e:
        rospy.logerr(f"Error: {e}")
        import traceback
        rospy.logerr(traceback.format_exc())
