import pandas as pd
import matplotlib.pyplot as plt

def visualize_physics():
    # 1. 读取数据
    try:
        df = pd.read_csv('simulation_day03.csv')
    except FileNotFoundError:
        print("错误：找不到 simulation_day03.csv 文件，请先运行 C++ 程序。")
        return

    # 设置画布大小
    plt.figure(figsize=(15, 10))
    plt.subplots_adjust(hspace=0.4)

    # --- 子图 1：运动轨迹 (X vs Y) ---
    plt.subplot(2, 2, 1)
    plt.plot(df['Pos_X'], df['Pos_Y'], 'b-', label='Trajectory')
    plt.scatter(df['Pos_X'].iloc[0], df['Pos_Y'].iloc[0], color='green', label='Start')
    plt.scatter(df['Pos_X'].iloc[-1], df['Pos_Y'].iloc[-1], color='red', label='End')
    plt.title('2D Trajectory (Parabolic Motion)')
    plt.xlabel('Position X (m)')
    plt.ylabel('Position Y (m)')
    plt.legend()
    plt.grid(True)

    # --- 子图 2：旋转角度变化 ---
    plt.subplot(2, 2, 2)
    plt.plot(df['Time'], df['Angle'], 'r-', label='Rotation (Rad)')
    plt.title('Rotation Over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (Radians)')
    plt.legend()
    plt.grid(True)

    # --- 子图 3：线速度变化 ---
    plt.subplot(2, 2, 3)
    plt.plot(df['Time'], df['Vel_X'], label='Vel X')
    plt.plot(df['Time'], df['Vel_Y'], label='Vel Y')
    plt.title('Linear Velocity Over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.legend()
    plt.grid(True)

    # --- 子图 4：综合角度与位置 (示意旋转) ---
    # 我们画几个点，用箭头表示物体的朝向
    plt.subplot(2, 2, 4)
    step = 15 # 每 15 帧画一个朝向箭头
    subset = df.iloc[::step]
    import numpy as np
    plt.quiver(subset['Pos_X'], subset['Pos_Y'], 
               np.cos(subset['Angle']), np.sin(subset['Angle']), 
               color='purple', scale=15, label='Orientation')
    plt.plot(df['Pos_X'], df['Pos_Y'], alpha=0.3)
    plt.title('Position and Orientation')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.grid(True)

    # 保存图片
    plt.savefig('physics_result.png')
    print("可视化完成！图片已保存为 physics_result.png")
    plt.show()

if __name__ == "__main__":
    visualize_physics()