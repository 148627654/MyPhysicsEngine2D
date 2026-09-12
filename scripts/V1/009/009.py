import pandas as pd
import matplotlib.pyplot as plt
import os

# 1. 加载数据
file_path = '009_multi.csv'
if not os.path.exists(file_path):
    print(f"Error: {file_path} not found!")
    exit()

df = pd.read_csv(file_path)

# 2. 创建画布
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
plt.style.use('bmh')

# --- 图表 1: X-Y 轨迹 (物理空间) ---
for body_id in df['BodyID'].unique():
    subset = df[df['BodyID'] == body_id]
    label_name = f"Body {body_id} ({subset['Shape'].iloc[0]})"
    
    # 绘制轨迹
    ax1.plot(subset['PosX'], subset['PosY'], label=label_name, alpha=0.7, linewidth=2)
    # 绘制终点
    ax1.scatter(subset['PosX'].iloc[-1], subset['PosY'].iloc[-1], marker='X', s=50)

# 绘制地面参考线 (地面位置在 -5, 高度 1, 所以顶面在 -4.5)
ax1.axhline(y=-4.5, color='r', linestyle='--', label='Ground Level')

ax1.set_title('Multi-Body Physical Trajectories (Day 09)', fontsize=14)
ax1.set_xlabel('Position X')
ax1.set_ylabel('Position Y')
ax1.legend(loc='upper right', bbox_to_anchor=(1.15, 1))
ax1.axis('equal') # 保证 X-Y 比例 1:1
ax1.grid(True)

# --- 图表 2: 垂直速度随时间变化 (证明冲量生效) ---
for body_id in df['BodyID'].unique():
    if body_id == 0: continue # 跳过静态地面
    subset = df[df['BodyID'] == body_id]
    ax2.plot(subset['Frame'], subset['VelY'], label=f'Body {body_id} VelY')

ax2.set_title('Vertical Velocity over Time (Impulse Response)', fontsize=14)
ax2.set_xlabel('Frame')
ax2.set_ylabel('Velocity Y')
ax2.axhline(y=0, color='black', linewidth=1)
ax2.legend()
ax2.grid(True)

plt.tight_layout()
output_img = 'day09_multi_test.png'
plt.savefig(output_img)
print(f"Visualization saved to: {output_img}")
plt.show()