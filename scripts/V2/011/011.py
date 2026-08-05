import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches
import numpy as np
import os
# 自动锁定脚本同级目录
base_dir = os.path.dirname(os.path.abspath(__file__))
csv_file = os.path.join(base_dir, "v2_011.csv")  # 先定义变量

df = pd.read_csv(csv_file)
print(df.head())

# 2. 设置画布
fig, ax = plt.subplots(figsize=(12, 6))
ax.set_aspect('equal')
ax.set_xlim(-7, 10)
ax.set_ylim(-3, 3)

# 颜色定义
COLOR_WALL = '#2c3e50'    # 墙壁（深色）
COLOR_BULLET = '#e74c3c'  # 子弹（红色）
COLOR_SWEPT = '#fab1a0'   # 扫掠区域（淡橘色）

def update(frame_idx):
    ax.clear()
    ax.set_xlim(-7, 10)
    ax.set_ylim(-3, 3)
    ax.grid(True, linestyle=':', alpha=0.5)
    
    # 获取当前帧数据
    curr_frame = df[df['Frame'] == frame_idx]
    
    # 获取子弹和墙的位置
    wall = curr_frame[curr_frame['BodyID'] == 0].iloc[0]
    bullet = curr_frame[curr_frame['BodyID'] == 1].iloc[0]
    
    # --- 绘制墙壁 (0.1 x 10) ---
    wall_rect = patches.Rectangle(
        (wall['PosX'] - 0.05, wall['PosY'] - 5), 0.1, 10,
        color=COLOR_WALL, label='Thin Wall (0.1m)'
    )
    ax.add_patch(wall_rect)

    # --- 绘制扫掠残影 (Swept AABB 可视化) ---
    # 如果不是第一帧，计算从上一帧到这一帧的扫掠范围
    if frame_idx > 0:
        prev_bullet = df[(df['Frame'] == frame_idx-1) & (df['BodyID'] == 1)].iloc[0]
        # 计算扫掠包围盒的宽度
        swept_width = abs(bullet['PosX'] - prev_bullet['PosX']) + 0.5 # 0.5是子弹宽度
        swept_x = min(bullet['PosX'], prev_bullet['PosX']) - 0.25
        
        swept_rect = patches.Rectangle(
            (swept_x, bullet['PosY'] - 0.25), swept_width, 0.5,
            color=COLOR_SWEPT, alpha=0.4, label='Swept AABB (BroadPhase)'
        )
        ax.add_patch(swept_rect)

    # --- 绘制子弹 (0.5 x 0.5) ---
    bullet_rect = patches.Rectangle(
        (bullet['PosX'] - 0.25, bullet['PosY'] - 0.25), 0.5, 0.5,
        color=COLOR_BULLET, edgecolor='black', linewidth=1, label='Bullet'
    )
    ax.add_patch(bullet_rect)

    ax.set_title(f"Day 11: Swept AABB Detection | Frame: {frame_idx}\nBullet Vel: 120m/s | Wall Thick: 0.1m", fontsize=12)
    ax.legend(loc='upper right')

# 3. 生成动画
frames = df['Frame'].unique()
print(f"Generating animation for {len(frames)} frames...")
ani = FuncAnimation(fig, update, frames=frames, interval=100)

# 4. 保存
output_gif = "v2_day11.gif"
ani.save(output_gif, writer='pillow', fps=10)
print(f"Success! Visualization saved to {output_gif}")
plt.show()