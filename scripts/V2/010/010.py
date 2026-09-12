import os
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches
import matplotlib.transforms as transforms
import numpy as np

# 获取当前py脚本所在文件夹绝对路径
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

def create_physics_gif(csv_name, output_gif, x_range=(-8, 12), y_range=(-3, 18)):
    # 拼接csv完整路径
    csv_path = os.path.join(BASE_DIR, csv_name)
    print(f"Processing {csv_path}...")
    try:
        df = pd.read_csv(csv_path)
    except Exception as e:
        print(f"Error: {e}")
        return

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_aspect('equal')
    
    # 自动识别地基尺寸
    is_tower = "tower" in csv_name

    def update(frame_idx):
        ax.clear()
        ax.set_xlim(*x_range)
        ax.set_ylim(*y_range)
        ax.grid(True, linestyle=':', alpha=0.4)
        
        frame_data = df[df['Frame'] == frame_idx]
        
        for _, row in frame_data.iterrows():
            b_id = int(row['BodyID'])
            pos_x, pos_y = row['PosX'], row['PosY']
            angle = np.degrees(row['Angle'])
            vx, vy = row['VelX'], row['VelY']
            
            # 尺寸定义
            if b_id == 0: # 地面
                w, h = 50.0 if not is_tower else 20.0, 2.0
            elif "chain" in csv_name and b_id == 6: # 子弹
                w, h = 0.5, 0.5
            else: # 标准箱子
                w, h = 1.0, 1.0
            
            # 状态判定
            is_sleeping = (vx == 0 and vy == 0 and b_id != 0)
            
            # 颜色
            if b_id == 0: color = '#2c3e50' # 地面
            elif ("chain" in csv_name and b_id == 6): color = '#e74c3c' # 子弹红色
            else: color = '#bdc3c7' if is_sleeping else '#3498db'

            # 绘制
            rect = patches.Rectangle(
                (pos_x - w/2, pos_y - h/2), w, h,
                linewidth=1, edgecolor='black', facecolor=color, alpha=0.8
            )
            t = transforms.Affine2D().rotate_deg_around(pos_x, pos_y, angle) + ax.transData
            rect.set_transform(t)
            ax.add_patch(rect)
            
            if is_sleeping:
                ax.text(pos_x, pos_y + h/2 + 0.2, "Zzz", fontsize=8, ha='center', color='gray')

        ax.set_title(f"Day 10: {csv_name} | Frame: {frame_idx}")

    frames = df['Frame'].unique()
    ani = FuncAnimation(fig, update, frames=frames, interval=33)
    output_full = os.path.join(BASE_DIR, output_gif)
    ani.save(output_full, writer='pillow', fps=30)
    print(f"Saved to {output_full}")
    plt.close()

# 只传文件名即可
create_physics_gif('v2_010_chain.csv', 'v2_010_chain.gif', x_range=(-6, 10), y_range=(-2, 4))
create_physics_gif('v2_010_tower.csv', 'v2_010_tower.gif', x_range=(-5, 5), y_range=(-2, 16))