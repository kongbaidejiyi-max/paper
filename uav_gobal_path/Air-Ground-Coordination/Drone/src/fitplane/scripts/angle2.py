#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
import threading
from fitplane.msg import GridPlaneInfo
import matplotlib
matplotlib.use('Qt5Agg')

class TraversabilityVisualizer:
    def __init__(self):
        rospy.init_node('traversability_visualizer', anonymous=True)

        # 初始化图形
        self.fig, self.axes = plt.subplots(1, 2, figsize=(16, 8))
        self.fig.suptitle('Traversability and Surface Angle Analysis\n(Click on any point to see details)', fontsize=16)

        titles = ['Traversability (Higher = Less Traversable)', 'Surface Angle (Higher = Steeper)']
        for ax, title in zip(self.axes, titles):
            ax.set_title(title, fontsize=12)
            ax.set_aspect('equal')
            ax.grid(True, alpha=0.3)
            ax.set_xlabel('X (m)', fontsize=10)
            ax.set_ylabel('Y (m)', fontsize=10)

        self.images = [None, None]
        self.colorbars = [None, None]
        self.current_data = [None, None]
        self.current_resolution = None
        self.current_origin = None
        self.width = 0
        self.height = 0

        self.info_text = self.fig.text(0.02, 0.02, 'Click on any point to see detailed information', 
                                       transform=self.fig.transFigure, 
                                       fontsize=10, verticalalignment='bottom',
                                       bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))

        self.fig.canvas.mpl_connect('button_press_event', self.on_click)

        rospy.Subscriber('grid_plane_info', GridPlaneInfo, self.callback)

        rospy.loginfo("Traversability Visualizer initialized")

        # 启动 matplotlib 的事件循环线程
        self.plot_thread = threading.Thread(target=self.plot_loop)
        self.plot_thread.daemon = True
        self.plot_thread.start()

    def plot_loop(self):
        rospy.loginfo("Plot thread started")
        plt.show()
        rospy.loginfo("Plot window closed, shutting down ROS...")
        rospy.signal_shutdown("User closed the plot window")

    def world_to_grid(self, x, y):
        if self.current_origin is None or self.current_resolution is None:
            return None, None
        grid_x = int(round((x - self.current_origin.x) / self.current_resolution))
        grid_y = int(round((y - self.current_origin.y) / self.current_resolution))
        return grid_x, grid_y

    def on_click(self, event):
        if event.inaxes is None or self.current_data[0] is None:
            return

        x, y = event.xdata, event.ydata
        grid_x, grid_y = self.world_to_grid(x, y)
        if grid_x is None or grid_y is None:
            return

        if not (0 <= grid_x < self.width and 0 <= grid_y < self.height):
            self.info_text.set_text(f'Clicked outside valid area\nClick position: ({x:.2f}, {y:.2f})')
            self.fig.canvas.draw_idle()
            return

        trav_val = self.current_data[0][grid_y, grid_x]
        angle_val = self.current_data[1][grid_y, grid_x]

        info_lines = [
            f"📍 CLICKED POINT DETAILS",
            f"World Position: ({x:.3f}, {y:.3f}) m",
            f"Grid Position: ({grid_x}, {grid_y})",
            f"Resolution: {self.current_resolution:.3f} m/cell",
            ""
        ]

        if not np.isnan(trav_val) and not np.isnan(angle_val):
            info_lines.append(f"🚶 Traversability: {trav_val:.6f}")
            info_lines.append(f"📐 Surface Angle: {angle_val:.2f}°")
            
            trav_desc = "🟢 Easy" if trav_val < 0.3 else "🟡 Moderate" if trav_val < 0.7 else "🔴 Difficult"
            angle_desc = "🟢 Flat" if angle_val < 15 else "🟡 Moderate slope" if angle_val < 45 else "🔴 Steep"
            
            info_lines.append(f"Assessment: {trav_desc}, {angle_desc}")
        else:
            info_lines.append("❌ No valid data at this position")

        self.info_text.set_text('\n'.join(info_lines))
        self.fig.canvas.draw_idle()

        print(f"\n{'='*50}")
        for line in info_lines:
            print(line)
        print(f"{'='*50}")

    def callback(self, msg):
        try:
            width, height = msg.width, msg.height
            if width <= 0 or height <= 0:
                return

            trav = np.array(msg.traversability).reshape(height, width).astype(float)
            angle = np.array(msg.plane_angle).reshape(height, width).astype(float)

            unknown_mask = (trav == -1)
            trav[unknown_mask] = np.nan
            angle[unknown_mask] = np.nan

            self.current_data = [trav, angle]
            self.current_resolution = msg.resolution
            self.current_origin = msg.origin
            self.width = width
            self.height = height

            extent = [msg.origin.x, msg.origin.x + width * msg.resolution,
                      msg.origin.y, msg.origin.y + height * msg.resolution]

            data_list = [trav, angle]
            cmaps = ['RdYlGn_r', 'coolwarm']
            vlims = [(0, 1), (0, 90)]
            colorbar_labels = ['Traversability (0=Easy, 1=Difficult)', 'Surface Angle (0=Flat, 90=Vertical)']

            for i, (ax, data, cmap, vlim, cb_label) in enumerate(zip(self.axes, data_list, cmaps, vlims, colorbar_labels)):
                vmin, vmax = vlim
                if self.images[i] is None:
                    self.images[i] = ax.imshow(data, origin='lower', extent=extent, cmap=cmap, vmin=vmin, vmax=vmax)
                    if self.colorbars[i]:
                        self.colorbars[i].remove()
                    self.colorbars[i] = plt.colorbar(self.images[i], ax=ax, shrink=0.8)
                    self.colorbars[i].set_label(cb_label, fontsize=9)
                else:
                    self.images[i].set_data(data)
                    self.images[i].set_extent(extent)
                    self.images[i].set_clim(vmin, vmax)

                ax.set_xlim(extent[0], extent[1])
                ax.set_ylim(extent[2], extent[3])

            self.info_text.set_text(f'Data updated: {width}×{height} grid\nClick anywhere on the plots for details')
            self.fig.canvas.draw_idle()

            rospy.loginfo(f"Updated visualization: {width}×{height} grid")

        except Exception as e:
            rospy.logerr(f"Visualization error: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())

    def run(self):
        rospy.spin()
        rospy.loginfo("ROS spin completed")

if __name__ == '__main__':
    try:
        visualizer = TraversabilityVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupted")
    except Exception as e:
        rospy.logerr(f"Error: {e}")
        import traceback
        rospy.logerr(traceback.format_exc())
