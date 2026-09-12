import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches
import matplotlib.transforms as transforms
import numpy as np

def create_physics_gif(csv_path, output_gif, x_range=(-10, 10), y_range=(-3, 8), platform_size=(5, 1)):
    print(f"Reading {csv_path}...")
    try:
        df = pd.read_csv(csv_path)
    except Exception as e:
        print(f"Error reading {csv_path}: {e}")
        return

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.set_aspect('equal')
    
    # 定义不同物体的尺寸映射 (Width, Height)
    # 根据你的 C++ 代码配置：
    # 001 测试中：地基 20x2, 箱子 1x1
    # 002 测试中：平台 5x1, 箱子 1x1
    size_map = {
        0: (20.0, 2.0) if "001" in csv_path else platform_size,
        1: (1.0, 1.0),
        2: (1.0, 1.0)
    }

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
            
            # 获取该 ID 对应的尺寸，默认为 1x1
            w, h = size_map.get(b_id, (1.0, 1.0))
            
            # 状态判定
            is_sleeping = (vx == 0 and vy == 0)
            
            # 颜色分配
            if b_id == 0:
                color = '#2c3e50' # 静态/地基
            else:
                color = '#bdc3c7' if is_sleeping else '#3498db'

            # 绘制矩形
            rect = patches.Rectangle(
                (pos_x - w/2, pos_y - h/2), w, h,
                linewidth=1, edgecolor='black', facecolor=color, alpha=0.8
            )
            
            # 旋转变换
            t = transforms.Affine2D().rotate_deg_around(pos_x, pos_y, angle) + ax.transData
            rect.set_transform(t)
            ax.add_patch(rect)
            
            # 状态标注
            if is_sleeping and b_id != 0:
                ax.text(pos_x, pos_y + h/2 + 0.3, "Zzz", fontsize=9, ha='center', color='gray')
            
            label = "Platform" if b_id == 0 else f"Box {b_id}"
            ax.text(pos_x, pos_y, label, fontsize=8, ha='center', va='center', color='white', fontweight='bold')

        ax.set_title(f"Day 09 Simulation: {csv_path}\nFrame: {frame_idx}")

    frames = df['Frame'].unique()
    ani = FuncAnimation(fig, update, frames=frames, interval=33)
    
    print(f"Saving {output_gif}...")
    ani.save(output_gif, writer='pillow', fps=30)
    plt.close()

# --- 生成第一个测试：悬空唤醒 ---
create_physics_gif(
    csv_path='v2_009_001.csv', 
    output_gif='v2_009_001.gif',
    x_range=(-5, 5), 
    y_range=(-3, 5)
)

# --- 生成第二个测试：类型转换 ---
create_physics_gif(
    csv_path='v2_009_002.csv', 
    output_gif='v2_009_002.gif',
    x_range=(-5, 5), 
    y_range=(-10, 5),
    platform_size=(5, 1)
)

print("All animations generated successfully!")