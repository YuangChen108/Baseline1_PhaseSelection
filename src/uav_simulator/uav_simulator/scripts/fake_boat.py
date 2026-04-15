#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import random
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

# 全局变量：当前目标点 (默认在原点)
target_x = 0.0
target_y = 0.0
has_goal = False

def goal_callback(msg):
    global target_x, target_y, has_goal
    target_x = msg.pose.position.x
    target_y = msg.pose.position.y
    has_goal = True
    rospy.loginfo("New Goal Received: x=%.2f, y=%.2f", target_x, target_y)

def fake_boat():
    rospy.init_node('fake_boat_node')

    rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)
    pub = rospy.Publisher('/target_pose', Odometry, queue_size=10)
    

    # 1. 读取速度参数
    velocity = rospy.get_param('~velocity', 1.0) 
    
    if rospy.has_param('~wave_omega'):
            wave_omega = rospy.get_param('~wave_omega')

    # 3. 读取海平面基准高度
    if rospy.has_param('/drone0/prediction/z_mean'):
        z_mean = rospy.get_param('/drone0/prediction/z_mean')
    elif rospy.has_param('prediction/z_mean'):
        z_mean = rospy.get_param('prediction/z_mean')
    else:
        z_mean = 0.0

    # 4. 读取海浪振幅
    wave_amp = rospy.get_param('~wave_amp', 0.8) 
    
    rospy.loginfo(f"Boat Params: Vel={velocity}, Freq={wave_omega}, Amp={wave_amp}, Z_Mean={z_mean}")
    
    curr_x = 0.0
    curr_y = 0.0
    curr_yaw = 0.0
    
    rate = rospy.Rate(50) # 50Hz
    start_time = rospy.Time.now().to_sec()
    time_offset = random.uniform(0.0, 100.0)

    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec()
        t = (current_time - start_time) + time_offset
        dt = 0.02 

        # === 1. 水平运动计算 ===
        vx = 0.0
        vy = 0.0
        if has_goal:
            dx = target_x - curr_x
            dy = target_y - curr_y
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist > 0.1:
                curr_yaw = math.atan2(dy, dx)
                step = velocity * dt
                if step > dist: step = dist
                curr_x += step * math.cos(curr_yaw)
                curr_y += step * math.sin(curr_yaw)
                
                # 计算世界坐标系下的水平速度分量
                vx = velocity * math.cos(curr_yaw)
                vy = velocity * math.sin(curr_yaw)

        # === 2. 海浪运动计算 (位置与速度) ===
        # Z 位置: z(t) = A * sin(wt) + z_mean
        z = wave_amp * math.sin(wave_omega * t) + z_mean
        
        # ✅ 核心修复：Z 速度 (对时间求导): vz(t) = A * w * cos(wt)
        vz = wave_amp * wave_omega * math.cos(wave_omega * t)

        # 船身摇晃
        roll = 0.1 * math.sin(0.5 * t)
        pitch = 0.05 * math.cos(0.4 * t)
        q = quaternion_from_euler(roll, pitch, curr_yaw)

        # === 3. 填充里程计消息 ===
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "world"

        odom.pose.pose.position.x = curr_x
        odom.pose.pose.position.y = curr_y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # ✅ 填充正确的三维速度
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz
        # 加上这行调试输出
        rospy.loginfo("Debug - Z: %.4f, VZ: %.4f", z, vz)
        pub.publish(odom)
        rate.sleep()

if __name__ == '__main__':
    try:
        fake_boat()
    except rospy.ROSInterruptException:
        pass