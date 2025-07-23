#!/usr/bin/env python3
# -*- coding=UTF-8 -*-
"""
版权所有 (c) 2024 [睿尔曼智能科技有限公司]。保留所有权利。
作者: Robert 时间: 2024/07/20

在满足以下条件的情况下，允许重新分发和使用源代码和二进制形式的代码，无论是否修改：
1. 重新分发的源代码必须保留上述版权声明、此条件列表和以下免责声明。
2. 以二进制形式重新分发的代码必须在随分发提供的文档和/或其他材料中复制上述版权声明、此条件列表和以下免责声明。

本软件由版权持有者和贡献者“按原样”提供，不提供任何明示或暗示的保证，
包括但不限于对适销性和特定用途适用性的暗示保证。
在任何情况下，即使被告知可能发生此类损害的情况下，
版权持有者或贡献者也不对任何直接的、间接的、偶然的、特殊的、惩罚性的或后果性的损害
（包括但不限于替代商品或服务的采购；使用、数据或利润的损失；或业务中断）负责，
无论是基于合同责任、严格责任还是侵权行为（包括疏忽或其他原因）。



示例代码通过订阅话题/servo_state接收舵机状态 通过话题/servo_control/move发布舵机id和舵机角度来控制舵机转动

示例用法：
>>> rospy.Subscriber("/servo_state", ServoAngle, servo_state_callback)
>>> pub = rospy.Publisher("/servo_control/move", ServoMove, queue_size=10)
"""


import rospy
from servo_ros.msg import ServoAngle, ServoMove


def servo_state_callback(data):
    # 打印收到的舵机状态信息
    rospy.loginfo(f"收到的舵机状态信息: {data}")

def listener_and_publisher():

    # 初始化ROS节点
    rospy.init_node('servo_state_listener_and_publisher', anonymous=True)

    # 订阅 /servo_state 话题
    rospy.Subscriber("/servo_state", ServoAngle, servo_state_callback)

    # 创建一个发布者对象，发布到 /servo_control/move 话题
    pub = rospy.Publisher("/servo_control/move", ServoMove, queue_size=10)

    # 设置发布频率
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        
        # 创建ServoMove消息并设置内容
        move_msg1 = ServoMove()
        move_msg1.servo_id = 1
        move_msg1.angle = servo1_angle
        
        move_msg2 = ServoMove()
        move_msg2.servo_id = 2
        move_msg2.angle = servo2_angle

        # 发布消息
        rospy.loginfo("发布舵机控制信息: %s", move_msg1)
        pub.publish(move_msg1)
        rospy.loginfo("发布舵机控制信息: %s", move_msg2)
        pub.publish(move_msg2)

        # 按照设定的频率等待
        rate.sleep()

if __name__ == '__main__':

    servo1_angle = 500 #舵机1设置的角度
    servo2_angle = 450 #舵机2设置的角度

    listener_and_publisher()

