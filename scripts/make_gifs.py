# -*- coding: utf-8 -*-
# make_gifs.py — 把 2Ddemo 的 CSV 轨迹渲染成 GIF
# 用法: python scripts/make_gifs.py
# 输出: 2Ddemo/*.gif
#   car_demo.gif（避震小车轨迹）、double_pendulum.gif（双摆混沌轨迹）
#   elevator_demo.gif（电梯楼层停靠）、crane_demo.gif（起重机变幅+滑轮组）
#   tank_demo.gif（战车翻坡上高原）、flail_demo.gif（链锤砸砖墙）
import csv
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.animation as anim
from matplotlib.patches import Rectangle, Circle as MplCircle, Polygon as MplPolygon

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
C_BRICK = "#b07a45"   # 砖块
C_ROPE = "#555555"    # 绳索
C_STATIC = "#9a9a9a"  # 静态构件（塔/导轨/地面）


def load_body_frames(csv_path):
    """解析 CSV: Frame -> [(BodyID, Shape, x, y, angle)]（按帧聚合）"""
    frames = {}
    with open(csv_path, encoding="utf-8") as f:
        rd = csv.reader(f)
        next(rd)
        for row in rd:
            fr = int(row[0])
            frames.setdefault(fr, []).append(
                (int(row[1]), row[2], float(row[3]), float(row[4]), float(row[5])))
    return frames


def _add_rot_rect(ax, x, y, w, h, ang, color, **kw):
    ax.add_patch(Rectangle((x - w / 2, y - h / 2), w, h,
                           angle=ang * 180 / math.pi, color=color,
                           rotation_point="center", **kw))


def save_gif(fig, update, sample, out_path, fps=30):
    ani = anim.FuncAnimation(fig, update, frames=sample, interval=100)
    ani.save(out_path, writer=anim.PillowWriter(fps=fps))
    plt.close(fig)
    print(f"[完成] {out_path}（{len(sample)} 帧, {os.path.getsize(out_path)//1024} KB）")


def make_car_gif():
    csv_path = os.path.join(DEMO, "car_demo.csv")
    if not os.path.exists(csv_path):
        print("[跳过] 未找到 car_demo.csv（先运行 CarDemo 生成）")
        return

    # 解析：Frame -> [(BodyID, Shape, x, y, angle)]
    frames = load_body_frames(csv_path)

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
                ax.add_patch(MplCircle((x, y), 0.5, facecolor=C_WHEEL, edgecolor="black", lw=1))
            elif bid == 11:
                _add_rot_rect(ax, x, y, 2.2, 0.25, ang, C_PLATE)
            elif bid == 12:
                _add_rot_rect(ax, x, y, 2.8, 0.5, ang, C_CHASSIS)
            else:
                w, h = DIMS[bid][1], DIMS[bid][2]
                _add_rot_rect(ax, x, y, w, h, ang, C_TRACK)
        ax.set_title(f"Car Demo — t={idx/60:.2f}s  (gold = trigger coins)")

    save_gif(fig, update, sample, os.path.join(DEMO, "car_demo.gif"))


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

    save_gif(fig, update, sample, os.path.join(DEMO, "double_pendulum.gif"))


# ============================================================
# V4 Day 12/13 新 Demo：电梯 / 起重机 / 战车 / 链锤
# ============================================================

def make_elevator_gif():
    """电梯：Prismatic 马达 + 楼层限位，乘客随轿厢平稳升降。
    BodyID: 0=井道导轨 1=轿厢 2=乘客（与 ElevatorDemo.cpp 创建顺序一致）"""
    csv_path = os.path.join(DEMO, "elevator_demo.csv")
    if not os.path.exists(csv_path):
        print("[跳过] 未找到 elevator_demo.csv（先运行 ElevatorDemo 生成）")
        return

    frames = load_body_frames(csv_path)
    sample = list(range(0, len(frames), 3))  # 600 帧抽 1/3

    fig, ax = plt.subplots(figsize=(6, 8), dpi=100)
    ax.set_aspect("equal")

    def update(idx):
        ax.clear()
        ax.set_facecolor("#f5f5f5")
        ax.grid(True, color="#dddddd", lw=0.5)
        ax.set_xlim(-5.0, 5.0)
        ax.set_ylim(-8.0, 8.0)
        # 楼层限位线
        for ly in (-5.0, 5.0):
            ax.plot([-5, 5], [ly, ly], "--", lw=1, color="#bbbbbb")
        for bid, shape, x, y, ang in frames[idx]:
            if bid == 0:
                _add_rot_rect(ax, x, y, 0.4, 16.0, ang, C_STATIC)
            elif bid == 1:
                _add_rot_rect(ax, x, y, 2.0, 0.5, ang, C_CHASSIS)
            elif bid == 2:
                _add_rot_rect(ax, x, y, 1.0, 1.0, ang, C_PLATE)
        ax.set_title(f"Elevator Demo — t={idx/60:.2f}s  (blue=car, orange=passenger)")

    save_gif(fig, update, sample, os.path.join(DEMO, "elevator_demo.gif"))


def make_crane_gif():
    """起重机：GearJoint 变幅（曲柄:吊臂 = 1:4）+ PulleyJoint 2:1 动滑轮。
    BodyID: 0=吊塔 1=曲柄轮 2=吊臂 3=配重 4=货物；绳端挂点 (8,8)"""
    csv_path = os.path.join(DEMO, "crane_demo.csv")
    if not os.path.exists(csv_path):
        print("[跳过] 未找到 crane_demo.csv（先运行 CraneDemo 生成）")
        return

    frames = load_body_frames(csv_path)
    sample = list(range(0, len(frames), 2))  # 240 帧抽 1/2
    PIVOT = (0.0, 8.0)     # 吊臂铰点
    HANG = (8.0, 8.0)      # 滑轮组挂点

    fig, ax = plt.subplots(figsize=(9, 7), dpi=100)
    ax.set_aspect("equal")

    def update(idx):
        ax.clear()
        ax.set_facecolor("#f5f5f5")
        ax.grid(True, color="#dddddd", lw=0.5)
        ax.set_xlim(-3.0, 12.0)
        ax.set_ylim(-3.0, 10.5)
        counter = cargo = None
        for bid, shape, x, y, ang in frames[idx]:
            if bid == 0:
                _add_rot_rect(ax, x, y, 0.5, 8.0, ang, C_STATIC)
            elif bid == 1:
                ax.add_patch(MplCircle((x, y), 0.3, facecolor=C_WHEEL, edgecolor="black", lw=1))
            elif bid == 2:
                _add_rot_rect(ax, x, y, 8.0, 0.4, ang, C_PLATE)
            elif bid == 3:
                counter = (x, y)
                ax.add_patch(MplCircle((x, y), 0.25, facecolor=C_ROD1, edgecolor="black", lw=1))
            elif bid == 4:
                cargo = (x, y)
                _add_rot_rect(ax, x, y, 1.0, 1.0, ang, C_CHASSIS)
        # 滑轮组绳（挂点 → 配重 / 货物）
        if counter:
            ax.plot([HANG[0], counter[0]], [HANG[1], counter[1]], "-", lw=1.5, color=C_ROPE)
        if cargo:
            ax.plot([HANG[0], cargo[0]], [HANG[1], cargo[1]], "-", lw=1.5, color=C_ROPE)
        ax.plot(*PIVOT, "o", ms=7, color="black")
        ax.plot(*HANG, "o", ms=6, color=C_ROPE)
        ax.set_title(f"Crane Demo — t={idx/60:.2f}s  (gear 1:4 + pulley 2:1)")

    save_gif(fig, update, sample, os.path.join(DEMO, "crane_demo.gif"))


# 梯形车身顶点（TankDemo.cpp 局部坐标，CCW）
TRAP_VERTS = [(-1.5, -0.4), (1.5, -0.4), (1.0, 0.4), (-1.0, 0.4)]


def make_tank_gif():
    """战车：双 WheelJoint 硬悬挂爬 20° 坡上高原 + 车顶炮塔限位旋转。
    BodyID: 0=平地 1=斜坡 2=高原 3=梯形车身(Polygon→CSV 记为 Box)
            4=左轮 5=右轮 6=炮塔"""
    csv_path = os.path.join(DEMO, "tank_demo.csv")
    if not os.path.exists(csv_path):
        print("[跳过] 未找到 tank_demo.csv（先运行 TankDemo 生成）")
        return

    frames = load_body_frames(csv_path)
    sample = list(range(0, len(frames), 2))  # 360 帧抽 1/2

    fig, ax = plt.subplots(figsize=(12, 5), dpi=100)
    ax.set_aspect("equal")

    def update(idx):
        ax.clear()
        ax.set_facecolor("#f5f5f5")
        ax.grid(True, color="#dddddd", lw=0.5)
        bodies = frames[idx]
        cam_x = next(b[2] for b in bodies if b[0] == 3)  # 跟随车身
        ax.set_xlim(cam_x - 11, cam_x + 11)
        ax.set_ylim(-2.5, 8.0)
        for bid, shape, x, y, ang in bodies:
            if bid == 0:
                _add_rot_rect(ax, x, y, 20.0, 1.0, ang, C_TRACK)
            elif bid == 1:
                _add_rot_rect(ax, x, y, 15.2, 0.5, ang, C_TRACK)
            elif bid == 2:
                _add_rot_rect(ax, x, y, 15.0, 1.0, ang, C_TRACK)
            elif bid == 3:
                c, s = math.cos(ang), math.sin(ang)
                pts = [(x + c * vx - s * vy, y + s * vx + c * vy) for vx, vy in TRAP_VERTS]
                ax.add_patch(MplPolygon(pts, closed=True, facecolor=C_CHASSIS,
                                        edgecolor="black", lw=1.2))
            elif bid in (4, 5):
                ax.add_patch(MplCircle((x, y), 0.4, facecolor=C_WHEEL, edgecolor="black", lw=1))
            elif bid == 6:
                _add_rot_rect(ax, x, y, 1.0, 0.3, ang, C_ROD1)
        ax.set_title(f"Tank Demo — t={idx/60:.2f}s  (WheelJoint ×2, 20° slope)")

    save_gif(fig, update, sample, os.path.join(DEMO, "tank_demo.gif"))


def make_flail_gif():
    """链锤：RopeJoint(4m) + MouseJoint 圆周蓄力 → 松手砸穿 20 块砖墙。
    BodyID: 0=悬挂点 1=铁球 2=鼠标关节宿主(屏幕外) 3..22=砖块(c*5+r)"""
    csv_path = os.path.join(DEMO, "flail_demo.csv")
    if not os.path.exists(csv_path):
        print("[跳过] 未找到 flail_demo.csv（先运行 FlailDemo 生成）")
        return

    frames = load_body_frames(csv_path)
    sample = list(range(0, len(frames), 3))  # 420 帧抽 1/3
    ANCHOR = (0.0, 8.0)

    fig, ax = plt.subplots(figsize=(8, 6), dpi=100)
    ax.set_aspect("equal")

    def update(idx):
        ax.clear()
        ax.set_facecolor("#f5f5f5")
        ax.grid(True, color="#dddddd", lw=0.5)
        ax.set_xlim(-4.0, 6.0)
        ax.set_ylim(-0.5, 9.5)
        ball = None
        for bid, shape, x, y, ang in frames[idx]:
            if bid == 0:
                _add_rot_rect(ax, x, y, 0.4, 0.4, ang, C_STATIC)
            elif bid == 1:
                ball = (x, y)
                ax.add_patch(MplCircle((x, y), 0.5, facecolor=C_WHEEL, edgecolor="black", lw=1.5))
            elif bid == 2:
                continue  # 屏幕外宿主
            else:
                _add_rot_rect(ax, x, y, 0.25, 0.25, ang, C_BRICK, edgecolor="#5d3a1a", lw=0.6)
        if ball:
            ax.plot([ANCHOR[0], ball[0]], [ANCHOR[1], ball[1]], "-", lw=2, color=C_ROPE)
        ax.plot(*ANCHOR, "o", ms=7, color="black")
        ax.set_title(f"Flail & Mouse Demo — t={idx/60:.2f}s  (release at 2.5s)")

    save_gif(fig, update, sample, os.path.join(DEMO, "flail_demo.gif"))


if __name__ == "__main__":
    make_car_gif()
    make_pendulum_gif()
    make_elevator_gif()
    make_crane_gif()
    make_tank_gif()
    make_flail_gif()
