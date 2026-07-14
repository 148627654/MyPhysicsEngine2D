import pandas as pd
import matplotlib.pyplot as plt
import os

# 1. 读取数据
file_path = '008.csv'
if not os.path.exists(file_path):
    print(f"错误：找不到文件 {file_path}")
    exit()

df = pd.read_csv(file_path)

# 2. 设置绘图风格
plt.style.use('seaborn-v0_8') # 或者使用 'ggplot'
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

# --- 图表 1: X坐标 随 时间(Frame) 的变化 ---
for body_id in df['BodyID'].unique():
    subset = df[df['BodyID'] == body_id]
    ax1.plot(subset['Frame'], subset['PosX'], label=f'Body {body_id} (PosX)', linewidth=2)

ax1.set_title('Body Position X over Time (Frame)', fontsize=14)
ax1.set_xlabel('Frame')
ax1.set_ylabel('Position X')
ax1.legend()
ax1.grid(True)

# --- 图表 2: X-Y 轨迹图 (本例中Y为0) ---
for body_id in df['BodyID'].unique():
    subset = df[df['BodyID'] == body_id]
    # 画出轨迹线
    ax2.plot(subset['PosX'], subset['PosY'], label=f'Body {body_id} Trajectory', alpha=0.5)
    # 画出起点和终点
    ax2.scatter(subset['PosX'].iloc[0], subset['PosY'].iloc[0], marker='o', s=100, label=f'Body {body_id} Start')
    ax2.scatter(subset['PosX'].iloc[-1], subset['PosY'].iloc[-1], marker='X', s=100, label=f'Body {body_id} End')

ax2.set_title('X-Y Trajectory', fontsize=14)
ax2.set_xlabel('Position X')
ax2.set_ylabel('Position Y')
ax2.set_ylim(-1, 1) # Y轴范围稍微拉开一点方便观察
ax2.legend()
ax2.grid(True)

plt.tight_layout()

# 3. 保存并展示
output_img = 'day08_visualization.png'
plt.savefig(output_img)
print(f"可视化图表已保存至: {output_img}")
plt.show()