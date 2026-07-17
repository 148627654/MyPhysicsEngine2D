import pandas as pd
import matplotlib.pyplot as plt

# 1. 加载数据
df = pd.read_csv('011.csv')
plank = df[df['BodyID'] == 1]

# 2. 创建画布
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))
plt.style.use('bmh')

# --- 子图 1: Y 坐标随时间变化 ---
ax1.plot(plank['Frame'], plank['PosY'], label='Box Center Y', color='blue')
ax1.axhline(y=-3.0, color='red', linestyle='--', label='Expected Rest Height')
ax1.axhline(y=-5.0, color='black', linewidth=2, label='Ground Center')
ax1.set_title('Day 11: Positional Stability Test', fontsize=14)
ax1.set_ylabel('Y Position')
ax1.legend()

# --- 子图 2: 速度分析 (平动 vs 转动) ---
ax2.plot(plank['Frame'], plank['VelY'], label='Linear Vel Y', alpha=0.8)
ax2.plot(plank['Frame'], plank['AngVel'], label='Angular Vel', alpha=0.8)
ax2.set_title('Velocity & Rotation Analysis', fontsize=14)
ax2.set_xlabel('Frame')
ax2.set_ylabel('Value')
ax2.legend()

plt.tight_layout()
plt.savefig('day11_test_result.png')
print("可视化已保存：output/day11_test_result.png")
plt.show()