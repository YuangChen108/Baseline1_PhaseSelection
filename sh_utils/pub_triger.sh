#!/bin/bash

echo "Sending TRACK Trigger Signal..."

# 1. 发送启动信号 /triger
# 修正了 frame_id 和 w=1.0
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

echo "Signal Sent! Drone should be in TRACK mode."

# 提示：如果你发现飞机切了模式但不动，是因为小车太远了。
# 可以在这里手动把小车瞬移过来（仅限仿真）：
# rostopic pub /initialpose ... (这里一般需要手点，脚本里不好写死)

wait;