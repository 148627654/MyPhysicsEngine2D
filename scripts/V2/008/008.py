import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches
import matplotlib.transforms as transforms
import numpy as np
import os

# 1. 读取数据
csv_file = 'v2_008.csv'
df = pd.read_csv(csv_file)

# 2. 设置画布
fig, ax = plt.subplots(figsize=(10, 6))
ax.set_aspect('equal')

# 动态计算边界或手动设置
ax.set_xlim(-8, 15)
ax.set_ylim(-3, 10)

def update(frame_idx):
    ax.clear()
    ax.set_aspect('equal')
    ax.set_xlim(-8, 15)
    ax.set_ylim(-3, 10)
    ax.grid(True, linestyle=':', alpha=0.6)
    
    frame_data = df[df['Frame'] == frame_idx]
    
    for _, row in frame_data.iterrows():
        b_id = int(row['BodyID'])
        pos_x, pos_y = row['PosX'], row['PosY']
        angle = np.degrees(row['Angle'])
        vx, vy = row['VelX'], row['VelY']
        
        # 状态判定逻辑
        is_sleeping = (vx == 0 and vy == 0 and b_id != 0)
        
        # 尺寸与颜色
        if b_id == 0: # 地面
            w, h = 40.0, 2.0
            color = '#2c3e50' # 深灰色
        elif b_id == 2: # 撞击者
            w, h = 1.0, 1.0
            color = '#e74c3c' # 红色
        else: # 1号箱子
            w, h = 1.0, 1.0
            color = '#bdc3c7' if is_sleeping else '#3498db' # 灰色(睡)或蓝色(醒)

        # 绘制
        rect = patches.Rectangle(
            (pos_x - w/2, pos_y - h/2), w, h,
            linewidth=1, edgecolor='black', facecolor=color, alpha=0.8
        )
        
        # 旋转
        t = transforms.Affine2D().rotate_deg_around(pos_x, pos_y, angle) + ax.transData
        rect.set_transform(t)
        ax.add_patch(rect)
        
        # 文本标注
        if is_sleeping:
            ax.text(pos_x, pos_y + 0.7, "Zzz", fontsize=10, ha='center', color='gray', fontweight='bold')
        
        label = "Ground" if b_id == 0 else f"ID:{b_id}"
        ax.text(pos_x, pos_y, label, fontsize=8, ha='center', va='center', color='white')

    ax.set_title(f"V2 Day 08: Sleep & Wakeup Simulation | Frame: {frame_idx}")

# 3. 生成动画
frames = df['Frame'].unique()
print(f"Total frames: {len(frames)}. Generating animation...")
ani = FuncAnimation(fig, update, frames=frames, interval=30)

# 4. 保存
output_gif = "v2_day08_test.gif"
ani.save(output_gif, writer='pillow', fps=30)
print(f"Success! GIF saved to {output_gif}")
plt.show()