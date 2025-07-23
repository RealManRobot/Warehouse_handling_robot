#!/usr/bin/env python3
# -*- coding=UTF-8 -*-
"""
版权所有 (c) 2024 [睿尔曼智能科技有限公司]。保留所有权利。
作者: Abner 时间: 2024/07/25

在满足以下条件的情况下，允许重新分发和使用源代码和二进制形式的代码，无论是否修改：
1. 重新分发的源代码必须保留上述版权声明、此条件列表和以下免责声明。
2. 以二进制形式重新分发的代码必须在随分发提供的文档和/或其他材料中复制上述版权声明、此条件列表和以下免责声明。

本软件由版权持有者和贡献者“按原样”提供，不提供任何明示或暗示的保证，
包括但不限于对适销性和特定用途适用性的暗示保证。
在任何情况下，即使被告知可能发生此类损害的情况下，
版权持有者或贡献者也不对任何直接的、间接的、偶然的、特殊的、惩罚性的或后果性的损害
（包括但不限于替代商品或服务的采购；使用、数据或利润的损失；或业务中断）负责，
无论是基于合同责任、严格责任还是侵权行为（包括疏忽或其他原因）。

此模块为测试识别功能的实例demo。

此模块主要用于测试识别结果的返回情况，通过发布/target_item话题将识别结果发布给订阅者，
订阅者通过该名称获取到相关识别结果数据并发布在/cabinet_widget/result话题，等待订阅。
此外，本demo提供/cabinet_widget/result订阅者来检测识别结果的数据。

示例用法：
>>> label_tester = AgvTester("label_demo_node")
>>> label_tester.pub_thread.start()    # 发布者线程启动
>>> rospy.spin()    # 启动监听函数
"""

import rospy    # ROS模块
import threading    # 多线程模块

from std_msgs.msg import String    # ROS字符串消息类型模块
from dual_arm_msgs.msg import Cabinet    # 自定义识别结果消息类型模块


class LabelTester:
    """识别结果测试模块
    
    通过订阅/cabinet_widget/result话题获取对应的msg数据并输出，
    从而测试相机识别部分的可行性。

    Args:
        node_name (str): 节点名称
        anonymous (bool, optional): 是否采用匿名。Defaults to False.

    Attributes:
        pub_target_item (rospy.Publisher): 获取检测结果的名称发布器
        pub_thread (threading.Thread): 独立线程获取发布者数据
    """

    def __init__(self, node_name: str, anonymous: bool=False):
        """构造函数，做类的初始化所用

        初始化节点名称；通过绑定成员函数确定导航是否成功；通过绑定线程发布识别目标点的名称

        Args:
            node_name (str): 节点名称
            anonymous (bool, optional): 是否采用匿名。Defaults to False.
        """

        rospy.init_node(name=node_name, anonymous=anonymous)    # 节点初始化
        rospy.Subscriber("/cabinet_widget/result", Cabinet, 
                         self.__cabinet_callback, queue_size=10)    # 绑定回调函数获取识别结果
        
        self.pub_target_item = rospy.Publisher("/target_item", 
                                               queue_size=10)    # 发布想要识别的目标点位的名称
        self.pub_thread = threading.Thread(target=self.run)    # 用多线程去发布标签名称，从而获取识别结果


    def __cabinet_callback(self, msg: Cabinet):
        """话题的数据订阅回调函数

        通过订阅/cabinet_widget/result"话题得到识别结果，
        包括识别物体位姿数据。

        Args:
            msg (Bool): 订阅到的数据
        """
        
        rospy.loginfo(f"Result:\n \
                      \tangle: { msg.angle }\n \
                      \tlable: { msg.label }\n \
                      \tpose:\n \
                      \t\tx: { msg.pose.position.x }\n \
                      \t\tx: { msg.pose.position.y }\n \
                      \t\tx: { msg.pose.position.z }\n ")

    def run(self):
        """线程函数，发布识别目标物的标签名称

        循环读取键盘输入并发布到/target_item话题
        """

        while not rospy.is_shutdown():
            point = str(input("请确定标签:\t"))    # 读取键盘输入
            self.pub_target_item.publish(point)    # 发布得到的数据


if __name__ == '__main__':

    label_tester = LabelTester("label_demo_node")    # 初始化节点名
    
    label_tester.pub_thread.start()    # 发布者线程启动
    
    rospy.spin()    # 启动监听函数
