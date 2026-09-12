import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches
import matplotlib.transforms as transforms
import numpy as np

# 1. 读取数据
csv_file = 'v2_007.csv'
df = pd.read_csv(csv_file)

# 2. 设置画布
fig, ax = plt.subplots(figsize=(10, 8))
ax.set_aspect('equal')
ax.grid(True, linestyle='--', alpha=0.6)

# 设置坐标轴范围 (根据你的场景调整)
ax.set_xlim(-15, 15)
ax.set_ylim(-2, 12)
ax.set_title("MyPhysicsEngine2D V2 - Island Simulation")

# 颜色映射：给不同的 BodyID 分配不同颜色
colors = ['#34495e', '#e74c3c', '#3498db', '#2ecc71', '#f1c40f', '#9b59b6']

def get_color(body_id):
    return colors[int(body_id) % len(colors)]

def update(frame_idx):
    ax.clear()
    ax.set_aspect('equal')
    ax.set_xlim(-15, 15)
    ax.set_ylim(-2, 12)
    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")
    ax.grid(True, linestyle='--', alpha=0.6)
    
    # 获取当前帧的所有物体
    frame_data = df[df['Frame'] == frame_idx]
    
    for _, row in frame_data.iterrows():
        pos_x = row['PosX']
        pos_y = row['PosY']
        angle = np.degrees(row['Angle']) # 弧度转角度
        shape_type = row['Shape']
        body_id = row['BodyID']
        
        # 假设：地面（ID 0）尺寸为 20x1，其余盒子为 1x1
        # 如果你在 C++ 中定义了不同尺寸，请在这里对应修改
        if body_id == 0:
            width, height = 20.0, 1.0
        else:
            width, height = 1.0, 1.0
            
        if shape_type == 'Box':
            # 创建矩形，注意 matplotlib 默认旋转中心是左下角
            # 我们需要将其偏移到物体中心
            rect = patches.Rectangle(
                (pos_x - width/2, pos_y - height/2), 
                width, height, 
                linewidth=1.5, 
                edgecolor='black', 
                facecolor=get_color(body_id),
                alpha=0.8
            )
            
            # 处理旋转
            t = transforms.Affine2D().rotate_deg_around(pos_x, pos_y, angle) + ax.transData
            rect.set_transform(t)
            ax.add_patch(rect)
            
        elif shape_type == 'Circle':
            radius = 0.5 # 默认半径
            circle = patches.Circle(
                (pos_x, pos_y), 
                radius, 
                linewidth=1.5, 
                edgecolor='black', 
                facecolor=get_color(body_id),
                alpha=0.8
            )
            ax.add_patch(circle)
            
        # 绘制 Body ID 标签 (可选)
        ax.text(pos_x, pos_y, str(int(body_id)), fontsize=8, ha='center', va='center')

    ax.set_title(f"Frame: {frame_idx}")

# 3. 创建动画
frames = df['Frame'].unique()
print(f"Generating animation for {len(frames)} frames...")

ani = FuncAnimation(fig, update, frames=frames, interval=33)

# 4. 保存为 GIF
# 注意：这需要系统中安装了 pillow 或 imagemagick
output_file = 'v2_007.gif'
ani.save(output_file, writer='pillow', fps=30)

print(f"Animation saved as {output_file}")
plt.show()