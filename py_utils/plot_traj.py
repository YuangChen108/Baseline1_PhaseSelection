#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import os

# 文件路径 (确保和你在 C++ 里写的一致)
PLAN_FILE = "/home/Quadrotor-Landing-with-Minco/planned_traj.csv"
REAL_FILE = "/home/Quadrotor-Landing-with-Minco/actual_traj.csv"

def main():
    if not os.path.exists(PLAN_FILE) or not os.path.exists(REAL_FILE):
        print("❌ 找不到 CSV 文件！请确保实验已经跑完，并且 C++ 已经生成了数据。")
        return

    # 1. 读取数据
    df_plan = pd.read_csv(PLAN_FILE)
    df_real = pd.read_csv(REAL_FILE)

    # 2. 时间对齐 (把实际轨迹的时间戳归零，方便和规划轨迹对比)
    if not df_real.empty:
        start_time = df_real['SysTime'].iloc[0]
        df_real['RelativeTime'] = df_real['SysTime'] - start_time

    # 3. 创建画布
    fig = plt.figure(figsize=(16, 7))
    fig.canvas.manager.set_window_title('UAV Landing Trajectory Analysis')

    # ================= 视图 1：3D 空间轨迹对比 =================
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.set_title("3D Trajectory: Planned vs Actual")
    
    # 绘制规划轨迹 (虚线)
    ax1.plot(df_plan['UAV_Plan_X'], df_plan['UAV_Plan_Y'], df_plan['UAV_Plan_Z'], 
             label='Planned Traj (MINCO)', color='blue', linestyle='--', linewidth=2)
    
    # 绘制实际飞行轨迹 (实线)
    ax1.plot(df_real['Real_UAV_X'], df_real['Real_UAV_Y'], df_real['Real_UAV_Z'], 
             label='Actual Traj (UAV)', color='red', linewidth=2)
    
    # 绘制小船移动轨迹 (绿线)
    ax1.plot(df_real['Real_Boat_X'], df_real['Real_Boat_Y'], df_real['Real_Boat_Z'], 
             label='Boat Traj', color='green', linewidth=2)

    # 画一个蓝色的透明海面 (Z=0) 方便观察是否穿模
    xx, yy = np.meshgrid(np.linspace(min(df_real['Real_UAV_X'].min(), df_plan['UAV_Plan_X'].min()), 
                                     max(df_real['Real_UAV_X'].max(), df_plan['UAV_Plan_X'].max()), 10),
                         np.linspace(min(df_real['Real_UAV_Y'].min(), df_plan['UAV_Plan_Y'].min()), 
                                     max(df_real['Real_UAV_Y'].max(), df_plan['UAV_Plan_Y'].max()), 10))
    zz = np.zeros_like(xx)
    ax1.plot_surface(xx, yy, zz, alpha=0.2, color='cyan')

    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z / Altitude (m)')
    ax1.legend()

    # ================= 视图 2：Z轴 (高度) 随时间/距离的变化 =================
    # 相比于 3D 图，高度剖面图能最直观地看出是谁挖了坑
    ax2 = fig.add_subplot(122)
    ax2.set_title("Altitude (Z-axis) Profile")
    
    # 绘制规划高度
    ax2.plot(df_plan['Time'], df_plan['UAV_Plan_Z'], 
             label='Planned Altitude', color='blue', linestyle='--', linewidth=2)
    
    # 绘制实际高度 (由于时间可能不完全同步，这里横轴用相对时间)
    ax2.plot(df_real['RelativeTime'], df_real['Real_UAV_Z'], 
             label='Actual Altitude', color='red', linewidth=2)
    
    # 绘制船只高度 (海浪波动)
    ax2.plot(df_real['RelativeTime'], df_real['Real_Boat_Z'], 
             label='Boat Deck Height', color='green', linewidth=2)

    # 标记海平面 Z=0
    ax2.axhline(0, color='cyan', linestyle='-', linewidth=2, alpha=0.5, label='Water Surface (Z=0)')

    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Altitude Z (m)')
    ax2.grid(True)
    ax2.legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()