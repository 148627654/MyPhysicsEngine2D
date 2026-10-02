import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches
import matplotlib.transforms as transforms
import numpy as np
# BASE_DIR = os.path.dirname(os.path.abspath(__file__))
# 1. 配置文件路径
csv_files = [
    'v2_012_circle_circle.csv',
    'v2_012_box_circle.csv',
    'v2_012_box_box.csv'
]
titles = ["Circle vs Circle", "Box vs Circle", "Box vs Box"]

# 2. 读取所有数据并对齐帧数
dataframes = [pd.read_csv(f) for f in csv_files]
max_frames = min(len(df['Frame'].unique()) for df in dataframes)

# 3. 设置三并排画布
fig, axes = plt.subplots(1, 3, figsize=(18, 6))

def draw_body(ax, row, color, is_bullet=False):
    shape = row['Shape']
    pos_x, pos_y = row['PosX'], row['PosY']
    angle = np.degrees(row['Angle'])
    b_id = row['BodyID']

    if shape == 'Circle':
        # 根据测试代码：目标圆半径1.0，子弹圆半径0.5
        radius = 1.0 if b_id == 0 else 0.5
        circle = patches.Circle((pos_x, pos_y), radius, color=color, alpha=0.7, ec='black')
        ax.add_patch(circle)
    else:
        # 根据测试代码：目标盒 2x10，子弹盒 1x1
        w, h = (2.0, 10.0) if b_id == 0 else (1.0, 1.0)
        rect = patches.Rectangle((pos_x - w/2, pos_y - h/2), w, h, 
                                 color=color, alpha=0.7, ec='black')
        # 处理旋转
        t = transforms.Affine2D().rotate_deg_around(pos_x, pos_y, angle) + ax.transData
        rect.set_transform(t)
        ax.add_patch(rect)

def update(frame_idx):
    for i in range(3):
        ax = axes[i]
        ax.clear()
        ax.set_aspect('equal')
        ax.set_xlim(-6, 4)
        ax.set_ylim(-3, 3)
        ax.grid(True, linestyle=':', alpha=0.5)
        ax.set_title(f"{titles[i]}\nFrame: {frame_idx}")
        
        # 绘制墙/目标位置参考线
        ax.axvline(x=0, color='gray', linestyle='--', alpha=0.3)

        # 获取该文件、该帧的数据
        df = dataframes[i]
        frame_data = df[df['Frame'] == frame_idx]

        for _, row in frame_data.iterrows():
            color = '#e74c3c' if row['BodyID'] == 1 else '#2c3e50'
            draw_body(ax, row, color)

def main():
    print("Generating combined GIF...")
    # interval 200ms 为了看清每一帧的跳跃
    ani = FuncAnimation(fig, update, frames=range(max_frames), interval=200)
    
    output_filename = "v2_day12.gif"
    ani.save(output_filename, writer='pillow')
    
    print(f"Success! Combined visualization saved as {output_filename}")
    plt.show()

if __name__ == "__main__":
    main()