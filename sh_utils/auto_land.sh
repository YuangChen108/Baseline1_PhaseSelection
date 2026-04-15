#!/bin/bash

# ==========================================
# 步骤 0: 激活小船移动！(让船往远处开)
# ==========================================
echo "[Script] 0. Sending move command to the boat..."

# 这里给小船发布一个 (x: 20, y: 10) 的目标点，让它动起来
# ⚠️ 注意：如果你的船不动，请检查这里接收目标点的 topic 是不是 /move_base_simple/goal 
# 如果你平时在 RViz 里是用 "2D Nav Goal" 点击让船跑的，那大概率就是这个 topic。
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp: now
  frame_id: 'world'
pose:
  position:
    x: 5.0
    y: 50.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" --once

echo "[Script] Boat is moving!"

# 稍微等 1 秒，让船有个起步加速的过程
sleep 1

# ==========================================
# 步骤 1: 发送 TRACK 信号 (让无人机飞过去伴飞)
# ==========================================
echo "[Script] 1. Sending TRACK Trigger..."

rostopic pub /triger geometry_msgs/PoseStamped "header:
  seq: 0
  stamp: now
  frame_id: 'world'
pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" --once

echo "[Script] TRACK command sent!"

# ==========================================
# 步骤 2: 🚨 等待伴飞稳定 (给无人机追击的时间)
# ==========================================
echo "[Script] Waiting for 8 seconds for the drone to catch up and align..."
# 因为船在移动，无人机需要先加速追赶，然后再减速匹配船速，8秒刚好能让它死死咬住甲板。
sleep 4

# ==========================================
# 步骤 3: 发送 LAND 信号 (化身斜向追踪导弹)
# ==========================================
echo "[Script] 2. Drone is locked on! Sending LAND Trigger..."

rostopic pub /land_triger geometry_msgs/PoseStamped "header:
  seq: 0
  stamp: now
  frame_id: 'world'
pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" --once

echo "[Script] LAND command sent! Watch the drone swoop in."
