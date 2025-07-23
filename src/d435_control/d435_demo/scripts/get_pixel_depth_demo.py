#!/usr/bin/env python3
# -*- coding=UTF-8 -*-
"""
版权所有 (c) 2024 [睿尔曼智能科技有限公司]。保留所有权利。
作者: Jehan 时间: 2024/07/20

在满足以下条件的情况下，允许重新分发和使用源代码和二进制形式的代码，无论是否修改：
1. 重新分发的源代码必须保留上述版权声明、此条件列表和以下免责声明。
2. 以二进制形式重新分发的代码必须在随分发提供的文档和/或其他材料中复制上述版权声明、此条件列表和以下免责声明。

本软件由版权持有者和贡献者“按原样”提供，不提供任何明示或暗示的保证，
包括但不限于对适销性和特定用途适用性的暗示保证。
在任何情况下，即使被告知可能发生此类损害的情况下，
版权持有者或贡献者也不对任何直接的、间接的、偶然的、特殊的、惩罚性的或后果性的损害
（包括但不限于替代商品或服务的采购；使用、数据或利润的损失；或业务中断）负责，
无论是基于合同责任、严格责任还是侵权行为（包括疏忽或其他原因）。

此模块为如何订阅到D435相机话题数据提供演示。

此模块提供[
    D435相机深度图片数据、D435相机RGB图片数据
]的功能订阅演示。通过简单的订阅者挨个实现相机相应图片的显示。

示例用法：
>>> rospy.Subscriber("/camera/color/image_raw", Image, color_callback, queue_size = 1)
>>> rospy.spin()
其他函数功能使用方式类似。
"""

import sys

import rospy

from d435_ros.msg import Pixel
from std_msgs.msg import Float32  


def get_depth_callback(data):

    rospy.loginfo(f"depth:{data}")

def pub_pixel():

    pixel = Pixel()
    pixel.x = pixel_x
    pixel.y = pixel_y

    pixel_sub.publish(pixel)

def main():

    pub_pixel()


if __name__ == '__main__':

    try:

        pixel_x = float(sys.argv[1])   #像素点x坐标值

        pixel_y = float(sys.argv[2])   #像素点y坐标值
    
    except IndexError:
        
        rospy.logerr("没有传入像素点数据")

    else:

        rospy.init_node('depth_listener', anonymous=True)

          
        pixel_sub = rospy.Publisher("/camera/pixel", Pixel,queue_size=10)   # 创建一个发布者（Publisher），用于向/camera/pixel话题发布Pixel类型的消息。

        depth_sub = rospy.Subscriber("/camera/depth",Float32,get_depth_callback,queue_size=10)#订阅当前状态（关节角度和位姿）

        
        rospy.sleep(1)

        main()

        rospy.sleep(1)