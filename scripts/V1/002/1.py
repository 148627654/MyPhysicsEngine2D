import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# 1. 读取数据
try:
    df = pd.read_csv('simulation_data.csv')
except FileNotFoundError:
    print("错误：找不到 simulation_data.csv 文件，请先运行 C++ 程序生成数据。")
    exit()

# 2. 创建画布
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

# --- 图表 1: 静态轨迹图 ---
ax1.plot(df['Pos_X'], df['Pos_Y'], label='Trajectory', color='blue', linestyle='--')
ax1.axhline(0, color='black', lw=1) # 画地平线
ax1.set_title("Static Trajectory (X vs Y)")
ax1.set_xlabel("Position X (m)")
ax1.set_ylabel("Position Y (m)")
ax1.grid(True)
ax1.legend()

# --- 图表 2: 动态动画 ---
ax2.set_xlim(df['Pos_X'].min() - 1, df['Pos_X'].max() + 1)
ax2.set_ylim(df['Pos_Y'].min() - 1, df['Pos_Y'].max() + 1)
ax2.axhline(0, color='red', lw=2, label='Ground') # 地面
ax2.set_title("Real-time Animation")
ax2.set_xlabel("Position X (m)")
ax2.grid(True)

point, = ax2.plot([], [], 'go', ms=10, label='RigidBody') # 绘制物体点
trail, = ax2.plot([], [], 'g-', alpha=0.3) # 绘制尾迹

def init():
    point.set_data([], [])
    trail.set_data([], [])
    return point, trail

def update(frame):
    # 更新物体位置
    x = df['Pos_X'].iloc[:frame]
    y = df['Pos_Y'].iloc[:frame]
    
    current_x = df['Pos_X'].iloc[frame]
    current_y = df['Pos_Y'].iloc[frame]
    
    point.set_data([current_x], [current_y])
    trail.set_data(x, y)
    return point, trail

# 创建动画 (每隔 20ms 更新一帧)
ani = FuncAnimation(fig, update, frames=len(df), init_func=init, blit=True, interval=20)

plt.tight_layout()
plt.savefig("physics_demo.png")
plt.show()

# 如果你想保存动画为 mp4 或 gif，可以取消下面这一行的注释 (需要安装 ffmpeg)
# ani.save('physics_demo.gif', writer='imagemagick')