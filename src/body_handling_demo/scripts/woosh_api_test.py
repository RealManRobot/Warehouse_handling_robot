#!/usr/bin/env python3
# -*- coding=UTF-8 -*-

import rospy
import warnings

class CustomLogger:
    def __init__(self):
        # 保存原始的 rospy.loginfo 函数
        self.original_loginfo = rospy.loginfo

        # 重载 rospy.loginfo
        rospy.loginfo = self.custom_loginfo

    # 自定义警告函数
    def custom_warn(self, msg):
        warnings.warn(f"自定义警告: { msg }")

    # 自定义的 loginfo 函数
    def custom_loginfo(self, msg):
        self.custom_warn(msg)  # 触发警告
        # self.original_loginfo(msg)  # 调用原始的 rospy.loginfo

# 测试
if __name__ == "__main__":
    # 初始化 ROS 节点
    rospy.init_node("test_node")

    # 创建 CustomLogger 实例，重载 rospy.loginfo
    logger = CustomLogger()

    # 调用 rospy.loginfo
    rospy.loginfo("这是一条 info 日志！")