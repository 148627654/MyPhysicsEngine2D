import pandas as pd
import matplotlib.pyplot as plt

# 1. 加载数据
try:
    df = pd.read_csv('012.csv')
except FileNotFoundError:
    df = pd.read_csv('012.csv')

# 提取不同测试目标的数据
slider = df[df['BodyID'] == 1]      # 摩擦力滑块
plank = df[df['BodyID'] == 2]       # 倾斜着陆木板
box_bottom = df[df['BodyID'] == 3]  # 堆叠底层
box_top = df[df['BodyID'] == 4]     # 堆叠顶层

# 2. 创建画布 (3个子图)
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 15))
plt.style.use('bmh')

# --- 子图 1: 摩擦力测试 (Slider VelX) ---
ax1.plot(slider['Frame'], slider['VelX'], color='red', linewidth=2, label='Slider VelX')
ax1.set_title('Experiment A: Friction Braking (Coulomb Friction)', fontsize=14)
ax1.set_ylabel('Horizontal Velocity (m/s)')
ax1.axhline(y=0, color='black', linestyle=':', alpha=0.5)
ax1.legend()
ax1.set_xlabel('Frame')

# --- 子图 2: 多点碰撞稳定性 (Plank Angle & AngVel) ---
# 左轴显示角度
ax2.plot(plank['Frame'], plank['Angle'], color='blue', label='Plank Angle (Rad)')
ax2.set_ylabel('Angle (Radians)', color='blue')
ax2.tick_params(axis='y', labelcolor='blue')

# 右轴显示角速度
ax2_right = ax2.twinx()
ax2_right.plot(plank['Frame'], plank['AngVel'], color='cyan', linestyle='--', alpha=0.6, label='Plank AngVel')
ax2_right.set_ylabel('Angular Velocity (Rad/s)', color='cyan')
ax2_right.tick_params(axis='y', labelcolor='cyan')

ax2.set_title('Experiment B: Multi-Point Landing (Stability)', fontsize=14)
ax2.axhline(y=0, color='black', linestyle='-', alpha=0.3)
lines, labels = ax2.get_legend_handles_labels()
lines2, labels2 = ax2_right.get_legend_handles_labels()
ax2.legend(lines + lines2, labels + labels2, loc='upper right')

# --- 子图 3: 堆叠稳定性测试 (PosY) ---
ax3.plot(box_bottom['Frame'], box_bottom['PosY'], label='Bottom Box Y', color='darkgreen')
ax3.plot(box_top['Frame'], box_top['PosY'], label='Top Box Y', color='limegreen')
ax3.set_title('Experiment C: Stacking Stability (Sequential Impulses)', fontsize=14)
ax3.set_ylabel('Y Position')
ax3.set_xlabel('Frame')
ax3.legend()

plt.tight_layout()
plt.savefig('day12_physical_validation.png')
print("分析完成！可视化图表已保存至：output/day12_physical_validation.png")
plt.show()