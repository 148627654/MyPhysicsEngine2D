import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle, Circle
import numpy as np

# 1. 加载数据
csv_file = 'v2_day6_sim.csv'
df = pd.read_csv(csv_file)

# 2. 设置画布
fig, ax = plt.subplots(figsize=(12, 8))
ax.set_aspect('equal')
ax.set_xlim(-20, 30)  # 根据你的子弹发射和墙体位置调整
ax.set_ylim(-30, 20)
ax.grid(True, linestyle='--', alpha=0.6)
ax.set_title("MyPhysicsEngine2D V2 - Day 6 Collision Stress Test")

# 存储每帧的图像对象
patches = []

def get_box_vertices(x, y, angle, w=1.0, h=1.0):
    """计算旋转后的矩形四个顶点坐标"""
    # 局部坐标
    corners = np.array([
        [-w/2, -h/2],
        [w/2, -h/2],
        [w/2, h/2],
        [-w/2, h/2]
    ])
    # 旋转矩阵
    c, s = np.cos(angle), np.sin(angle)
    R = np.array([[c, -s], [s, c]])
    # 变换到世界坐标
    return corners @ R.T + [x, y]

def update(frame_idx):
    ax.clear()
    ax.set_aspect('equal')
    ax.set_xlim(-20, 25)
    ax.set_ylim(-40, 15)
    ax.grid(True, linestyle='--', alpha=0.5)
    
    # 提取当前帧数据
    frame_data = df[df['Frame'] == frame_idx]
    ax.set_title(f"Frame: {frame_idx} | Bodies: {len(frame_data)}")

    for _, row in frame_data.iterrows():
        if row['Shape'] == 'Box':
            # 绘制方块 (假设宽1.0, 高1.0)
            verts = get_box_vertices(row['PosX'], row['PosY'], row['Angle'], 1.0, 1.0)
            poly = plt.Polygon(verts, closed=True, color='royalblue', edgecolor='black', alpha=0.8)
            ax.add_patch(poly)
        else:
            # 绘制圆 (假设半径0.5)
            circ = Circle((row['PosX'], row['PosY']), 0.5, color='crimson', edgecolor='black', zorder=10)
            ax.add_patch(circ)
            
    return []

# 3. 创建动画
frames = df['Frame'].unique()
print(f"Total frames found: {len(frames)}. Rendering...")

ani = animation.FuncAnimation(fig, update, frames=len(frames), interval=16, blit=False)

# 4. 保存视频 (需要 ffmpeg)
# 如果没有安装 ffmpeg，可以改为 ani.save('sim.gif', writer='pillow')
try:
    ani.save('physics_sim_v2.mp4', writer='ffmpeg', fps=60)
    print("Success: 'physics_sim_v2.mp4' has been generated.")
except Exception as e:
    print(f"Error saving video: {e}")
    print("Trying to save as GIF instead...")
    ani.save('physics_sim_v2.gif', writer='pillow', fps=30)

plt.show()