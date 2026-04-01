import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Arc
import warnings
warnings.filterwarnings('ignore')

def draw_triangle_by_angles(angle_A, angle_B, angle_C, side_length=10):
    """
    根据三个角度绘制三角形
    
    参数:
    angle_A, angle_B, angle_C: 三个角的度数
    side_length: 参考边长（用于确定三角形大小）
    
    返回:
    顶点坐标和角度信息
    """
    
    # 检查角度和是否为180度
    total_angle = angle_A + angle_B + angle_C
    if abs(total_angle - 180) > 0.001:
        print(f"警告：三角形内角和应为180°，当前为{total_angle:.2f}°")
        print("自动调整角C使内角和为180°")
        angle_C = 180 - angle_A - angle_B
        print(f"调整后的角C = {angle_C:.2f}°")
    
    # 转换为弧度
    A_rad = np.radians(angle_A)
    B_rad = np.radians(angle_B)
    C_rad = np.radians(angle_C)
    
    # 设置第一个顶点A在原点，第二个顶点B在x轴上
    A = np.array([0, 0])
    B = np.array([side_length, 0])
    
    # 使用正弦定理计算第三个顶点C的位置
    # 在三角形ABC中，边c对应角C，边a对应角A，边b对应角B
    # c/sin(C) = a/sin(A) = b/sin(B)
    
    # 已知边c = AB = side_length
    # 计算边a = BC 和边b = AC
    c = side_length  # 边AB
    a = c * np.sin(A_rad) / np.sin(C_rad)  # 边BC
    b = c * np.sin(B_rad) / np.sin(C_rad)  # 边AC
    
    # 计算C点坐标
    # C点到A点的距离为b，与x轴的夹角为angle_A
    C = np.array([b * np.cos(A_rad), b * np.sin(A_rad)])
    
    return A, B, C, (angle_A, angle_B, angle_C), (a, b, c)

def plot_triangle(A, B, C, angles, sides, title="三角形"):
    """绘制三角形"""
    
    plt.figure(figsize=(12, 8))
    
    # 绘制三角形
    triangle_x = [A[0], B[0], C[0], A[0]]
    triangle_y = [A[1], B[1], C[1], A[1]]
    
    plt.plot(triangle_x, triangle_y, 'b-', linewidth=2, label='三角形边')
    plt.fill(triangle_x[:-1], triangle_y[:-1], alpha=0.2, color='lightblue')
    
    # 标记顶点
    plt.scatter([A[0], B[0], C[0]], [A[1], B[1], C[1]], 
                color='red', s=80, zorder=5)
    
    # 标注顶点和角度
    offset = 0.5
    plt.annotate(f'A\n∠A={angles[0]:.1f}°', A, 
                xytext=(A[0]-offset, A[1]-offset), 
                fontsize=12, color='red', ha='center')
    plt.annotate(f'B\n∠B={angles[1]:.1f}°', B, 
                xytext=(B[0]+offset, B[1]-offset), 
                fontsize=12, color='red', ha='center')
    plt.annotate(f'C\n∠C={angles[2]:.1f}°', C, 
                xytext=(C[0], C[1]+offset), 
                fontsize=12, color='red', ha='center')
    
    # 标注边长
    mid_AB = (A + B) / 2
    mid_BC = (B + C) / 2
    mid_AC = (A + C) / 2
    
    plt.annotate(f'c={sides[2]:.2f}', mid_AB, 
                xytext=(mid_AB[0], mid_AB[1]-0.3), 
                fontsize=10, color='blue', ha='center')
    plt.annotate(f'a={sides[0]:.2f}', mid_BC, 
                xytext=(mid_BC[0]+0.3, mid_BC[1]), 
                fontsize=10, color='blue', ha='center')
    plt.annotate(f'b={sides[1]:.2f}', mid_AC, 
                xytext=(mid_AC[0]-0.3, mid_AC[1]), 
                fontsize=10, color='blue', ha='center')
    
    # 绘制角度弧线
    def draw_angle_arc(vertex, point1, point2, angle_deg, radius=1):
        # 计算两个向量的角度
        v1 = point1 - vertex
        v2 = point2 - vertex
        
        start_angle = np.degrees(np.arctan2(v1[1], v1[0]))
        end_angle = np.degrees(np.arctan2(v2[1], v2[0]))
        
        # 确保弧线方向正确
        if end_angle < start_angle:
            end_angle += 360
        
        arc = Arc(vertex, 2*radius, 2*radius, 
                 angle=0, theta1=start_angle, theta2=end_angle,
                 color='red', linewidth=2)
        plt.gca().add_patch(arc)
    
    # 绘制角度弧线
    draw_angle_arc(A, B, C, angles[0], 0.8)
    draw_angle_arc(B, C, A, angles[1], 0.8)
    draw_angle_arc(C, A, B, angles[2], 0.8)
    
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.title(f'{title}\n角度: A={angles[0]:.1f}°, B={angles[1]:.1f}°, C={angles[2]:.1f}°', 
              fontsize=14)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    
    # 显示三角形信息
    angle_sum = sum(angles)
    triangle_type = ""
    if all(angle < 90 for angle in angles):
        triangle_type = "锐角三角形"
    elif any(angle > 90 for angle in angles):
        triangle_type = "钝角三角形"
    elif any(abs(angle - 90) < 0.001 for angle in angles):
        triangle_type = "直角三角形"
    
    plt.text(0.02, 0.98, f'三角形类型: {triangle_type}\n内角和: {angle_sum:.1f}°', 
             transform=plt.gca().transAxes, fontsize=10, 
             verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat'))
    
    plt.tight_layout()
    plt.show()

# 示例使用
print("=== 三角形绘制器 ===")
print("可以绘制任意指定三个角度的三角形")
A, B, C, angles, sides = draw_triangle_by_angles(10, 160, 10)
plot_triangle(A, B, C, angles, sides, "等边三角形")