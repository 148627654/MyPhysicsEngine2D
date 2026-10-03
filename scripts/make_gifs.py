# -*- coding: utf-8 -*-
# make_gifs.py — 把 2Ddemo 的 CSV 轨迹渲染成 GIF（Day 12/13 样例可视化）
# 用法: python scripts/make_gifs.py
# 输出: 2Ddemo/car_demo.gif（避震小车轨迹）、2Ddemo/double_pendulum.gif（双摆混沌轨迹）
import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.animation as anim
from matplotlib.patches import Rectangle, Circle as MplCircle

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEMO = os.path.join(ROOT, "2Ddemo")

# ---------- 颜色 ----------
C_TRACK = "#8a8a8a"   # 赛道/桥体
C_WHEEL = "#d62728"   # 车轮
C_PLATE = "#ff7f0e"   # 副车架
C_CHASSIS = "#1f77b4" # 车身
C_COIN = "#ffd700"    # 金币
C_ROD1 = "#2ca02c"    # 摆杆 1
C_ROD2 = "#d62728"    # 摆杆 2


def make_car_gif():
    csv_path = os.path.join(DEMO, "car_demo.csv")
    if not os.path.exists(csv_path):
        print("[跳过] 未找到 car_demo.csv（先运行 CarDemo 生成）")
        return

    # 解析：Frame -> [(BodyID, Shape, x, y, angle)]
    frames = {}
    with open(csv_path, encoding="utf-8") as f:
        rd = csv.reader(f)
        next(rd)
        for row in rd:
            fr = int(row[0])
            frames.setdefault(fr, []).append(
                (int(row[1]), row[2], float(row[3]), float(row[4]), float(row[5])))

    # 形状尺寸表（与 CarDemo.cpp 的创建顺序一致；CSV 不存尺寸，只能对照源码）
    DIMS = {
        0: ("box", 40.0, 1.0),
        1: ("box", 1.8, 0.15), 2: ("box", 1.8, 0.15),
        3: ("box", 2.0, 0.15), 4: ("box", 2.0, 0.15),
        5: ("box", 1.6, 0.15), 6: ("box", 1.6, 0.15),
        7: ("box", 26.0, 0.5), 8: ("box", 60.0, 1.0),
        9: ("circle", 0.5, 0.0), 10: ("circle", 0.5, 0.0),
        11: ("box", 2.2, 0.25), 12: ("box", 2.8, 0.5),
    }
    COIN_IDS = set(range(13, 23))

    sample = list(range(0, len(frames), 3))  # 600 帧抽 1/3
    fig, ax = plt.subplots(figsize=(9, 4.5), dpi=100)
    ax.set_aspect("equal")

    def update(idx):
        ax.clear()
        ax.set_facecolor("#f5f5f5")
        ax.grid(True, color="#dddddd", lw=0.5)
        bodies = frames[idx]
        # 跟随车身（BodyID=12）的摄像机，纵向固定能看到全程爬坡
        cam_x = next(b[2] for b in bodies if b[0] == 12)  # 元组 (id, shape, x, y, ang)：x 在索引 2
        ax.set_xlim(cam_x - 14, cam_x + 14)
        ax.set_ylim(-2.5, 12.0)
        for bid, shape, x, y, ang in bodies:
            if bid in COIN_IDS:
                ax.add_patch(Rectangle((x - 0.2, y - 0.2), 0.4, 0.4,
                                       color=C_COIN, alpha=0.9, rotation_point="center"))
            elif bid in (9, 10):
                ax.add_patch(MplCircle((x, y), 0.5, color=C_WHEEL, edgecolor="black", lw=1))
            elif bid == 11:
                ax.add_patch(Rectangle((x - 1.1, y - 0.125), 2.2, 0.25, angle=ang * 180 / 3.14159,
                                       color=C_PLATE, rotation_point="center"))
            elif bid == 12:
                ax.add_patch(Rectangle((x - 1.4, y - 0.25), 2.8, 0.5, angle=ang * 180 / 3.14159,
                                       color=C_CHASSIS, rotation_point="center"))
            else:
                w, h = DIMS[bid][1], DIMS[bid][2]
                ax.add_patch(Rectangle((x - w / 2, y - h / 2), w, h,
                                       angle=ang * 180 / 3.14159, color=C_TRACK, rotation_point="center"))
        ax.set_title(f"Car Demo — t={idx/60:.2f}s  (gold = trigger coins)")

    ani = anim.FuncAnimation(fig, update, frames=sample, interval=100)
    out = os.path.join(DEMO, "car_demo.gif")
    ani.save(out, writer=anim.PillowWriter(fps=30))
    plt.close(fig)
    print(f"[完成] {out}（{len(sample)} 帧, {os.path.getsize(out)//1024} KB）")


def make_pendulum_gif():
    csv_path = os.path.join(DEMO, "double_pendulum.csv")
    if not os.path.exists(csv_path):
        print("[跳过] 未找到 double_pendulum.csv（先运行 DoublePendulum 生成）")
        return

    ANCHOR = (0.0, 6.0)
    rows = []
    with open(csv_path, encoding="utf-8") as f:
        rd = csv.reader(f)
        next(rd)
        for row in rd:
            rows.append(tuple(float(v) for v in row))  # frame,x1,y1,vx1,vy1,x2,y2,vx2,vy2

    sample = list(range(0, len(rows), 6))  # 1200 帧抽 1/5
    trail = []  # 末端轨迹（渐隐尾迹）

    fig, ax = plt.subplots(figsize=(6.5, 6.5), dpi=100)
    ax.set_aspect("equal")

    def update(idx):
        ax.clear()
        ax.set_facecolor("#f5f5f5")
        ax.grid(True, color="#dddddd", lw=0.5)
        ax.set_xlim(-4.6, 4.6)
        ax.set_ylim(0.4, 7.0)
        _, x1, y1, _, _, x2, y2, _, _ = rows[idx]
        # 杆端 = 锚点 + 2×(杆心 − 锚点)（杆心在半长 1.0 处，杆长 2.0）
        tip1 = (ANCHOR[0] + 2 * (x1 - ANCHOR[0]), ANCHOR[1] + 2 * (y1 - ANCHOR[1]))
        tip2 = (tip1[0] + 2 * (x2 - tip1[0]), tip1[1] + 2 * (y2 - tip1[1]))
        trail.append(tip2)
        if len(trail) > 80:
            trail.pop(0)
        # 混沌尾迹（越旧越淡）
        for k, (tx, ty) in enumerate(trail[:-1]):
            ax.plot(tx, ty, ".", ms=2.2, color=C_ROD2,
                    alpha=0.04 + 0.5 * (k / max(1, len(trail))))
        ax.plot([ANCHOR[0], tip1[0]], [ANCHOR[1], tip1[1]], "-", lw=6, color=C_ROD1, solid_capstyle="round")
        ax.plot([tip1[0], tip2[0]], [tip1[1], tip2[1]], "-", lw=6, color=C_ROD2, solid_capstyle="round")
        ax.plot(*ANCHOR, "o", ms=8, color="black")
        ax.plot(tip1[0], tip1[1], "o", ms=7, color="#2ca02c", mec="black")
        ax.plot(tip2[0], tip2[1], "o", ms=7, color="#d62728", mec="black")
        ax.set_title(f"Double Pendulum Chaos — frame {rows[idx][0]:.0f} ({rows[idx][0]/60:.2f}s)")

    ani = anim.FuncAnimation(fig, update, frames=sample, interval=100)
    out = os.path.join(DEMO, "double_pendulum.gif")
    ani.save(out, writer=anim.PillowWriter(fps=30))
    plt.close(fig)
    print(f"[完成] {out}（{len(sample)} 帧, {os.path.getsize(out)//1024} KB）")


if __name__ == "__main__":
    make_car_gif()
    make_pendulum_gif()
