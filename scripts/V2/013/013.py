import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches
import numpy as np

# 1. 加载数据
csv_file = 'v2_013.csv'
df = pd.read_csv(csv_file)

# 2. 画布设置
fig, ax = plt.subplots(figsize=(12, 5))
ax.set_aspect('equal')
ax.set_xlim(-6, 2)
ax.set_ylim(-2, 2)

# 颜色
COLOR_WALL = '#2c3e50'
COLOR_BULLET = '#e74c3c'
COLOR_TRAIL = '#fab1a0'

def update(frame_idx):
    ax.clear()
    ax.set_xlim(-6, 2)
    ax.set_ylim(-2, 2)
    ax.grid(True, linestyle=':', alpha=0.6)
    
    # 获取当前帧
    data = df[df['Frame'] == frame_idx]
    if data.empty: return
    
    # --- 绘制静态薄墙 (X=0, W=0.1) ---
    wall = patches.Rectangle((-0.05, -10), 0.1, 20, color=COLOR_WALL, label='Thin Wall (0.1m)')
    ax.add_patch(wall)

    # --- 绘制子弹 (ID: 1) ---
    bullet_row = data[data['BodyID'] == 1].iloc[0]
    px, py = bullet_row['PosX'], bullet_row['PosY']
    vx = bullet_row['VelX']
    
    # 绘制子弹本体
    bullet = patches.Rectangle((px - 0.25, py - 0.25), 0.5, 0.5, 
                               color=COLOR_BULLET, ec='black', lw=1, label='Bullet')
    ax.add_patch(bullet)
    
    # --- 绘制历史轨迹 (残影) ---
    history = df[(df['Frame'] <= frame_idx) & (df['BodyID'] == 1)]
    if len(history) > 1:
        ax.plot(history['PosX'], history['PosY'], color=COLOR_TRAIL, linestyle='--', alpha=0.6, lw=1)

    # 状态标注
    status = "REBOUNDING" if vx < 0 else "APPROACHING"
    ax.set_title(f"Day 13 CCD: High Speed Interception\nFrame: {frame_idx} | PosX: {px:.4f} | State: {status}")
    ax.legend(loc='upper left')

# 3. 生成动画
frames = df['Frame'].unique()
print(f"Generating animation for {len(frames)} frames...")
ani = FuncAnimation(fig, update, frames=frames, interval=200) # 慢放观察

# 4. 保存
output_gif = "v2_day13.gif"
ani.save(output_gif, writer='pillow', fps=10)
print(f"Success! Visualization saved as {output_gif}")
plt.show()