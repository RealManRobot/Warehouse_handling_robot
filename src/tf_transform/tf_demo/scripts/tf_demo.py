#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-
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

此模块用于获取识别目标位置基于机械臂基坐标系的位姿。

此模块首先通过建立camera_link坐标系基于head_link_roll坐标系的位姿关系，从而成功建立起坐标树。
之后通过客户端的请求，调用t2b_tf_server_callback回调函数，
得到识别物体/rm_target坐标系基于/base_link的位姿关系并反馈，
最终用以机械臂做出规划。

示例用法：
通过设置客户端去请求t2b_single_arm_server的服务便可获得最终的坐标关系。
"""

import rospy
from tf_msgs.srv import Target2Base, Target2BaseRequest, Target2BaseResponse

def target2base_response():
    """请求 /t2b_single_arm_server 服务并打印响应结果。
    
    Returns:
        Target2BaseResponse: 服务的响应对象。
    """
    # 创建服务代理，指向 /t2b_single_arm_server 服务
    target2base_service = rospy.ServiceProxy("/t2b_single_arm_server", Target2Base)
    
    # 创建请求对象
    req = Target2BaseRequest()
    
    # 发送服务请求并接收响应
    res = target2base_service(req)
    
    # 打印服务响应的内容
    print("The Pose of target to base link...", res.pose)
    
    return res  # 返回响应对象

if __name__ == "__main__":
    # 初始化ROS节点
    rospy.init_node("target2base_service_server")
    
    # 等待服务 /t2b_single_arm_server 变为可用
    rospy.wait_for_service('/t2b_single_arm_server')
    
    # 调用函数发送服务请求并获取响应
    response = target2base_response()
    
    # 可以选择在这里处理响应对象 response
    # 例如，根据响应中的位姿信息进行进一步的操作
    
    # 保持节点运行，直到被关闭
    rospy.spin()