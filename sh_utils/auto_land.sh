#!/bin/bash


# ==========================================
# 步骤 0: 启动小车 (让小车往 x=10, y=0 的地方跑)
# ==========================================
echo "[Script] 0. Moving the CAR..."

# 这里的 /move_base_simple/goal 是 RViz 那个绿色箭头的默认话题
# x: 10.0 表示让小车往前跑 10 米
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp: now
  frame_id: 'world'
pose:
  position:
    x: 5.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" --once

echo "[Script] Car is moving!"

# ==========================================
# 步骤 1: 发送 TRACK 信号 (开始追踪)
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
# 步骤 2: 等待 2 秒 (给飞机一点时间去调整姿态)
# ==========================================
echo "[Script] Waiting for 2 seconds..."
sleep 2

# ==========================================
# 步骤 3: 发送 LAND 信号 (触发降落)
# ==========================================
echo "[Script] 2. Time is up! Sending LAND Trigger..."

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

echo "[Script] LAND command sent! Watch the drone."