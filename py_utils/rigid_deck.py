#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from nav_msgs.msg import Odometry
import copy

class RigidDeckEnforcer:
    def __init__(self):
        rospy.init_node('rigid_deck_enforcer')
        
        # ================= 配置 =================
        self.drone_topic_in = "/drone0/odom"       # 原始飞机里程计
        self.boat_topic_in  = "/target_pose"       # 船的里程计
        self.drone_topic_out= "/drone0/odom_rigid" # 修正后的里程计 (RViz看这个!)
        
        self.deck_thickness = 0.15 # 允许飞机陷入甲板的深度阈值
        self.landed_z_offset = 0.05 # 降落后让飞机停在板子上方多少米 (脚架高度)
        # ========================================

        self.boat_odom = None
        self.is_landed = False
        
        # 订阅
        rospy.Subscriber(self.boat_topic_in, Odometry, self.boat_cb)
        rospy.Subscriber(self.drone_topic_in, Odometry, self.drone_cb)
        
        # 发布
        self.pub = rospy.Publisher(self.drone_topic_out, Odometry, queue_size=10)
        
        rospy.loginfo("🛡️ Rigid Deck Enforcer Started! Please visualize: " + self.drone_topic_out)

    def boat_cb(self, msg):
        self.boat_odom = msg

    def drone_cb(self, msg):
        if self.boat_odom is None:
            return

        # 复制一份消息准备修改
        rigid_msg = copy.deepcopy(msg)
        
        # 1. 获取高度
        drone_z = msg.pose.pose.position.z
        boat_z = self.boat_odom.pose.pose.position.z
        
        # 2. 计算相对高度
        rel_z = drone_z - boat_z

        # 3. 刚体碰撞逻辑
        # 如果飞机低于甲板一定程度，或者已经被标记为降落
        if rel_z < self.deck_thickness:
            self.is_landed = True
        
        # 如果还在高空，解除锁定（可选，防止误判）
        if rel_z > 0.5:
            self.is_landed = False

        # 4. 执行“锁定”
        if self.is_landed:
            # 【位置锁定】：强制把飞机拉回甲板表面
            rigid_msg.pose.pose.position.z = boat_z + self.landed_z_offset
            
            # 【水平跟随】：(可选) 如果你想模拟飞机由于摩擦力跟着船跑
            # 简单的做法是：当接触时，如果飞机没动力，应该会有个相对静止的视觉效果
            # 这里我们只修 Z 轴，水平方向通常 FSM 切到 GLUED 模式会自己跟
            
            # 【速度清零】：降落了垂直速度就是船的速度
            rigid_msg.twist.twist.linear.z = self.boat_odom.twist.twist.linear.z

        # 5. 发布修正后的里程计
        self.pub.publish(rigid_msg)

if __name__ == '__main__':
    try:
        RigidDeckEnforcer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass