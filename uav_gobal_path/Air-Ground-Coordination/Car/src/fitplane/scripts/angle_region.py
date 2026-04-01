#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from fitplane.msg import GridPlaneInfo
import matplotlib
from matplotlib.widgets import RectangleSelector
matplotlib.use('Qt5Agg')

class TraversabilityAngleVisualizer:
    def __init__(self):
        rospy.init_node('traversability_angle_visualizer', anonymous=True)
        
        # 创建1x2布局，并排显示两个图
        self.fig, self.axes = plt.subplots(1, 2, figsize=(16, 8))
        self.fig.suptitle('Traversability and Surface Angle Analysis (Drag to zoom, Right-click for details)', fontsize=16)
        
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
        self.current_data = [None, None]
        self.current_extent = None
        self.current_resolution = None
        self.current_origin = None
        
        # 区域选择器
        self.selectors = [None, None]
        self.zoom_history = [[], []]  # 存储缩放历史
        
        # 添加文本显示框
        self.info_text = self.fig.text(0.02, 0.98, 'Instructions:\n- Drag to select zoom area\n- Right-click for point details\n- Press "r" to reset zoom\n- Press "b" to go back', 
                                      transform=self.fig.transFigure, 
                                      fontsize=10, verticalalignment='top',
                                      bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
        
        # 连接事件
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        
        # 订阅话题
        rospy.Subscriber('grid_plane_info', GridPlaneInfo, self.callback_dual_display)
        
        plt.tight_layout()
        rospy.loginfo("Enhanced Traversability and Angle Visualizer initialized")
        rospy.loginfo("Controls: Drag=Zoom, Right-click=Details, R=Reset, B=Back")
    
    def setup_selectors(self):
        """设置区域选择器"""
        for i, ax in enumerate(self.axes):
            if self.selectors[i] is not None:
                self.selectors[i].disconnect_events()
            
            self.selectors[i] = RectangleSelector(
                ax, 
                lambda eclick, erelease, idx=i: self.on_select(eclick, erelease, idx),
                useblit=True,
                button=[1],  # 只响应左键
                minspanx=5, minspany=5,
                spancoords='pixels',
                interactive=True
            )
    
    def on_select(self, eclick, erelease, ax_index):
        """区域选择回调"""
        x1, y1 = eclick.xdata, eclick.ydata
        x2, y2 = erelease.xdata, erelease.ydata
        
        if None in [x1, y1, x2, y2]:
            return
        
        # 确保坐标顺序正确
        xmin, xmax = min(x1, x2), max(x1, x2)
        ymin, ymax = min(y1, y2), max(y1, y2)
        
        # 保存当前视图到历史
        ax = self.axes[ax_index]
        current_xlim = ax.get_xlim()
        current_ylim = ax.get_ylim()
        self.zoom_history[ax_index].append((current_xlim, current_ylim))
        
        # 应用缩放
        ax.set_xlim(xmin, xmax)
        ax.set_ylim(ymin, ymax)
        
        # 显示选中区域的统计信息
        self.show_region_stats(xmin, xmax, ymin, ymax)
        
        self.fig.canvas.draw_idle()
        rospy.loginfo(f"Zoomed to region: ({xmin:.2f}, {ymin:.2f}) to ({xmax:.2f}, {ymax:.2f})")
    
    def show_region_stats(self, xmin, xmax, ymin, ymax):
        """显示选中区域的统计信息"""
        if self.current_data[0] is None:
            return
        
        # 计算区域内的网格范围
        grid_x1, grid_y1 = self.world_to_grid(xmin, ymin)
        grid_x2, grid_y2 = self.world_to_grid(xmax, ymax)
        
        if None in [grid_x1, grid_y1, grid_x2, grid_y2]:
            return
        
        # 确保网格坐标在有效范围内
        height, width = self.current_data[0].shape
        grid_x1 = max(0, min(grid_x1, width-1))
        grid_x2 = max(0, min(grid_x2, width-1))
        grid_y1 = max(0, min(grid_y1, height-1))
        grid_y2 = max(0, min(grid_y2, height-1))
        
        # 提取区域数据
        x_start, x_end = min(grid_x1, grid_x2), max(grid_x1, grid_x2)
        y_start, y_end = min(grid_y1, grid_y2), max(grid_y1, grid_y2)
        
        trav_region = self.current_data[0][y_start:y_end+1, x_start:x_end+1]
        angle_region = self.current_data[1][y_start:y_end+1, x_start:x_end+1]
        
        # 计算统计信息
        trav_valid = trav_region[~np.isnan(trav_region)]
        angle_valid = angle_region[~np.isnan(angle_region)]
        
        info_lines = [
            f"Selected Region: ({xmin:.1f}, {ymin:.1f}) to ({xmax:.1f}, {ymax:.1f})",
            f"Grid cells: {len(trav_valid)} valid points",
        ]
        
        if len(trav_valid) > 0:
            info_lines.extend([
                f"Traversability: mean={np.mean(trav_valid):.3f}, std={np.std(trav_valid):.3f}",
                f"  Range: [{np.min(trav_valid):.3f}, {np.max(trav_valid):.3f}]"
            ])
        
        if len(angle_valid) > 0:
            info_lines.extend([
                f"Surface Angle: mean={np.mean(angle_valid):.1f}°, std={np.std(angle_valid):.1f}°",
                f"  Range: [{np.min(angle_valid):.1f}°, {np.max(angle_valid):.1f}°]"
            ])
            
            # 角度分类统计
            flat_count = np.sum(angle_valid < 15)
            moderate_count = np.sum((angle_valid >= 15) & (angle_valid < 45))
            steep_count = np.sum(angle_valid >= 45)
            info_lines.append(f"  Flat: {flat_count}, Moderate: {moderate_count}, Steep: {steep_count}")
        
        info_lines.extend([
            "",
            "Controls: Drag=Zoom, Right-click=Details",
            "Press 'r' to reset, 'b' to go back"
        ])
        
        self.info_text.set_text('\n'.join(info_lines))
    
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
        
        # 右键显示详细信息
        if event.button == 3:  # 右键
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
            
            # 获取周围3x3区域的数据
            self.show_neighborhood_data(x, y)
    
    def show_neighborhood_data(self, x, y):
        """显示点击位置周围的数据"""
        grid_x, grid_y = self.world_to_grid(x, y)
        if grid_x is None or grid_y is None:
            return
        
        height, width = self.current_data[0].shape
        
        # 获取3x3邻域
        info_lines = [f"Center Position: ({x:.2f}, {y:.2f}) m, Grid: ({grid_x}, {grid_y})"]
        info_lines.append("3x3 Neighborhood Data:")
        info_lines.append("Traversability | Angle")
        
        for dy in range(-1, 2):
            for dx in range(-1, 2):
                gx, gy = grid_x + dx, grid_y + dy
                if 0 <= gx < width and 0 <= gy < height:
                    trav_val = self.current_data[0][gy, gx]
                    angle_val = self.current_data[1][gy, gx]
                    
                    if not np.isnan(trav_val) and not np.isnan(angle_val):
                        marker = " * " if dx == 0 and dy == 0 else "   "
                        info_lines.append(f"{marker}{trav_val:.3f}     | {angle_val:.1f}°")
                    else:
                        marker = " * " if dx == 0 and dy == 0 else "   "
                        info_lines.append(f"{marker}  N/A      |  N/A")
        
        # 中心点评估
        center_trav = self.current_data[0][grid_y, grid_x]
        center_angle = self.current_data[1][grid_y, grid_x]
        
        if not np.isnan(center_trav):
            if center_trav < 0.3:
                trav_desc = "Easy"
            elif center_trav < 0.7:
                trav_desc = "Moderate"
            else:
                trav_desc = "Difficult"
            info_lines.append(f"Assessment: {trav_desc}")
        
        if not np.isnan(center_angle):
            if center_angle < 15:
                angle_desc = "Flat"
            elif center_angle < 45:
                angle_desc = "Moderate slope"
            else:
                angle_desc = "Steep"
            info_lines.append(f"Slope: {angle_desc}")
        
        info_lines.extend([
            "",
            "Controls: Drag=Zoom, Right-click=Details",
            "Press 'r' to reset, 'b' to go back"
        ])
        
        self.info_text.set_text('\n'.join(info_lines))
        self.fig.canvas.draw_idle()
    
    def on_key_press(self, event):
        """键盘事件处理"""
        if event.key == 'r':
            # 重置所有缩放
            for i, ax in enumerate(self.axes):
                if self.current_extent:
                    ax.set_xlim(self.current_extent[0], self.current_extent[1])
                    ax.set_ylim(self.current_extent[2], self.current_extent[3])
                self.zoom_history[i].clear()
            self.info_text.set_text('Zoom reset\n\nControls:\n- Drag to select zoom area\n- Right-click for point details\n- Press "r" to reset zoom\n- Press "b" to go back')
            self.fig.canvas.draw_idle()
            rospy.loginfo("Zoom reset")
            
        elif event.key == 'b':
            # 返回上一个缩放级别
            for i, ax in enumerate(self.axes):
                if self.zoom_history[i]:
                    xlim, ylim = self.zoom_history[i].pop()
                    ax.set_xlim(xlim)
                    ax.set_ylim(ylim)
            self.fig.canvas.draw_idle()
            rospy.loginfo("Zoom back")
            
        elif event.key == 'h':
            help_text = """
            Controls:
            - Drag with left mouse: Select area to zoom
            - Right-click: Show 3x3 neighborhood data
            - 'r' key: Reset zoom to full view
            - 'b' key: Go back to previous zoom level
            - 's' key: Save current data to files
            """
            print(help_text)
            
        elif event.key == 's':
            # 保存数据
            if self.current_data[0] is not None:
                np.savetxt('/tmp/traversability.txt', self.current_data[0], fmt='%.4f')
                np.savetxt('/tmp/surface_angle.txt', self.current_data[1], fmt='%.2f')
                rospy.loginfo("Data saved to /tmp/traversability.txt and /tmp/surface_angle.txt")
    
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
            cmaps = ['RdYlGn_r', 'coolwarm']
            vlims = [(0, 1), (0, 90)]
            colorbar_labels = [
                'Traversability Value\n(0=Easy, 1=Difficult)',
                'Surface Angle (degrees)\n(0=Flat, 90=Vertical)'
            ]
            
            # 更新每个子图
            for i, (ax, data, cmap, vlim, cb_label) in enumerate(zip(self.axes, data_list, cmaps, vlims, colorbar_labels)):
                
                # 计算颜色范围
                valid_data = data[~np.isnan(data)]
                if len(valid_data) > 0:
                    vmin, vmax = vlim
                    if i == 1:  # 角度图
                        vmax = 90
                else:
                    vmin, vmax = vlim
                
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
                    if i == 1:
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
                    
                    if i == 0:
                        title_text = (f'Traversability (Drag to zoom, Right-click for details)\n'
                                    f'Cells: {count}, Mean: {mean_val:.3f}, '
                                    f'Range: [{min_val:.3f}, {max_val:.3f}]')
                    else:
                        title_text = (f'Surface Angle (Drag to zoom, Right-click for details)\n'
                                    f'Cells: {count}, Mean: {mean_val:.1f}°, '
                                    f'Range: [{min_val:.1f}°, {max_val:.1f}°]')
                        
                        flat_count = np.sum(valid_data < 15)
                        moderate_count = np.sum((valid_data >= 15) & (valid_data < 45))
                        steep_count = np.sum(valid_data >= 45)
                        
                        title_text += f'\nFlat(<15°): {flat_count}, Moderate(15-45°): {moderate_count}, Steep(≥45°): {steep_count}'
                    
                    ax.set_title(title_text, fontsize=9)
                else:
                    titles = ['Traversability\nNo valid data', 'Surface Angle\nNo valid data']
                    ax.set_title(titles[i], fontsize=10)
            
            # 设置区域选择器
            self.setup_selectors()
            
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
            rospy.spin()
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
