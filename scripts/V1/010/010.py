import pandas as pd
import matplotlib.pyplot as plt

# 1. 读取数据
df = pd.read_csv('010.csv')
plank = df[df['BodyID'] == 1]

# 2. 画图
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))
plt.style.use('seaborn-v0_8')

# --- 子图 1: 角速度随时间变化 (证明碰撞产生旋转) ---
ax1.plot(plank['Frame'], plank['AngVel'], color='red', linewidth=2, label='Angular Velocity')
ax1.axhline(y=0, color='black', linestyle='--')
ax1.set_title('Angular Velocity Spike at Collision', fontsize=14)
ax1.set_xlabel('Frame')
ax1.set_ylabel('rad/s')
ax1.legend()

# --- 子图 2: 角度随时间变化 (证明物体在翻滚) ---
ax2.plot(plank['Frame'], plank['Angle'], color='blue', linewidth=2, label='Angle')
ax2.set_title('Rotation Angle (Orientation Change)', fontsize=14)
ax2.set_xlabel('Frame')
ax2.set_ylabel('Radians')
ax2.legend()

plt.tight_layout()
plt.savefig('day10_rotation_proof.png')
print("可视化已保存：output/day10_rotation_proof.png")
plt.show()