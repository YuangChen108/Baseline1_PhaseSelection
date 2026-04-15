#!/bin/bash
echo "[ROS Nuke] 开始清理所有残留的 ROS 和仿真进程..."

# 1. 无情绞杀所有带 ros 名字的进程（包括 rosmaster, roscore, rosout）
pkill -9 -f ros
# 2. 绞杀所有 Nodelet 僵尸管理器
pkill -9 -f nodelet
# 3. 绞杀可能残留的规划器和控制器
pkill -9 -f planning
pkill -9 -f so3_
# 4. 绞杀假船 Python 进程
pkill -9 -f fake_boat.py
# 5. 绞杀 Rviz
pkill -9 -f rviz
# 6. 清除 ROS 磁盘缓存日志（防止硬盘塞满导致卡顿）
rosclean purge -y

echo "[ROS Nuke] 清理完毕！系统已恢复绝对纯净。"