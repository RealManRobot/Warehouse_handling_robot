#!/usr/bin/env python3.8
# _*_ coding: utf-8 _*_
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

此模块将相机帧数据进行识别检测并与订阅的数据相匹配得到想要的数据通过话题发布出来。

此模块将通过realsense对象把D435数据对齐后的相机帧进行识别，并获取到识别结果中的label标签属性，
与此同时订阅话题/target_item中的目标label属性，并与上面的进行匹配，
最终将得到想要的具有label属性数据通过/cabinet_widget/result话题发布出来。

示例用法：
>>> main(pt: bool, conf: float, nc: int)
"""

import cv2    # OpenCV图片处理模块
import math    # 数学模块
import rospy    # ROS模块
import subprocess    # 连接输入/输出/错误管道
import numpy as np    # 数据处理模块
import tf.transformations    # tf坐标处理模块
from std_msgs.msg import String    # ROS的String消息类型

import custom_import    # 自定义引用模块
from camera import realsense    # D435相机驱动模块
from cam_Inference.solver import *    # 对相机进行计算
from cam_Inference.yolo_inference import Detect    # 识别模块

label_name = "JiaoHuanJi"    # 初始化标签（待检测）


def list_sinks():
    """获取特定类型的音频输出设备信息
    
    Returns:
        list: 包含特定类型的音频输出设备信息的列表
    """
    
    result = subprocess.run(['pactl', 'list', 'short', 'sinks'], 
                            stdout=subprocess.PIPE)    # 根据 sink_type 参数选择 pactl 命令的类型，捕获输出

    sinks = result.stdout.decode('utf-8').strip().split('\n')    # 解码为 utf-8 格式的字符串，
                                                                 # 并去除首尾空白字符
                                                                 # 按行分割输出，得到一个包含所有设备信息的列表
    sink_list = []    # 初始化一个空列表，用于存储设备信息
    for sink in sinks:
        sink_details = sink.split('\t')    # 使用 '\t' 分割每一行，得到一个包含设备详细信息的列表
        sink_list.append((sink_details[0], sink_details[1]))
    
    return sink_list    # 返回包含设备信息的列表


def set_default_sink(sink_index: str):
    """设置默认的音频输出设备，并把所有音频输入设备移动到这个默认输出设备上
    
    Args:
        sink_index (str): 要设置为默认的音频输出设备的索引号
    """

    subprocess.run(['pactl', 'set-default-sink', sink_index])    # 设置指定索引的音频输出设备为默认设备
    result = subprocess.run(['pactl', 'list', 'short', 'sink-inputs'], 
                            stdout=subprocess.PIPE)    # 执行 pactl 命令获取当前所有的音频输入设备列表
    inputs = result.stdout.decode('utf-8').strip().split('\n')    # 将输出解码为 UTF-8 字符串，
                                                                  # 并按行分割成列表
    for input in inputs:
        input_details = input.split('\t')    # 分割每一行以获取音频输入设备的详细信息
        subprocess.run(['pactl', 'move-sink-input', 
                        input_details[0], sink_index])    # 使用 pactl 命令将输入设备移动到新设置的默认输出设备上


def play_start():
    """播放启动音频
    
    函数首先查找带有USB名称的音频输出设备，如果找到则将其设置为默认设备，
    然后播放指定的MP3文件
    """

    sinks = list_sinks()    # 获取所有音频输出设备列表

    if not sinks:    # 判断是否有音频输出设备
        print("No audio sinks found.")
        return

    target_sink_index = None    # 初始化目标音频输出设备的索引

    for sink_index, sink_name in sinks:    # 寻找包含 "USB" 名称的设备
        if "USB" in sink_name:
            target_sink_index = sink_index    # 找到目标设备，记录索引
            break
    
    # 如果找到了目标音频输出设备，将其设置为默认并播放音频
    if target_sink_index is not None:
        set_default_sink(target_sink_index)    # 设置默认音频输出设备
        print(f"Switched to sink with 'USB' in the name: { sink_name }")
    else:
        print("No sink with 'USB' in the name found.")
    

def adjust_white_balance(image, alpha=1.0, beta=0.0):
    """调整图像的白平衡。

    Args:
        image: 输入图像
        alpha: 对比度缩放因子，默认为1.0（不进行缩放）
        beta: 亮度缩放因子，默认为0.0（不进行缩放）

    Returns:
        Any: 调整后的图像
    """
    adjusted_image = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
    return adjusted_image
    

def target_item_callback(msg: String):
    """目标的识别回调函数
    
    话题/target_item的回调函数，
    将得到的标签名称消息赋值给全局变量label_name

    Args:
        msg (String): 识别得到的物体名称
    """
    
    try:
        global label_name
        label_name = msg.data
        print(label_name)
    except Exception as e:
        rospy.logerr(e)
        
        
def main(pt, conf, nc):
    """主函数
    
    启动 ROS节点，订阅/target_item话题，
    并在循环中处理相机数据、识别物体并发布识别结果

    Args:
        pt (bool): 是否使用ONNX模型
        conf (float): YOLO模型置信度
        nc (int): YOLO模型类别数
    """

    solver = Solver()    # 实例化图片计算对象
    camera = realsense.RealSenseCamera("152222071683")    # 实例化D435相机驱动对象
    camera.start_camera()    # 启动相机
    detect = Detect()    # 实例化识别器

    rospy.init_node('cabinet_pub', anonymous=True)    # 初始化ROS节点，设置成匿名
    rospy.Subscriber('/target_item', String, 
                     target_item_callback, 
                     queue_size=1)    # 订阅者，订阅要识别的标签名称
    # 设置发布频率
    rate = rospy.Rate(10)  # 10hz

    # 根据是否使用pt模型加载不同的权重文件
    if pt:
        #yolo_weights = 'CD_pth/10500_50.pt'  改成自己的模型
        yolo_weights = custom_import.PT_PAYH + '/files_pth/Body_Handly_Robot.pt'
    else:
        yolo_weights = custom_import.PT_PAYH + '/files_pth/10500_50.onnx'

    solver_weights = custom_import.PT_PAYH + '/files_pth/CDNet.pth'
    
    # 生成模型
    model, solver_weights = detect.gen_model(tag=pt, yolo_weights=yolo_weights, solver=solver,
                                             solver_weights_path=solver_weights)
    # time.sleep(2.0)
    flag_i = 0
    global label_name
    flag_play = 0

    while True:
        color_img, depth_img, _, point_cloud, depth_frame = camera.read_align_frame()    # 读取对齐的相机帧
        color_img = cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR)    # 颜色空间转换
        deep_data3 = color_img
            

        #
        # deep_data_depth_esi = solver.test(img=color_img)
        # deep_data3 = cv2.cvtColor(deep_data_depth_esi, cv2.COLOR_GRAY2BGR)
        if pt:
            results = model(color_img, deep_data3, conf=conf)    # 使用pt模型进行推理

            annotated_frame, obj_img = detect.backward_handle_output(pt, results, color_img, depth_img,
                                                                     solver_weights, nc=nc, input=label_name)
        else:
            annotated_frame, mapped_depth = detect.backward_handle_output(pt, model, color_img, depth_img,
                                                                          solver_weights, nc=nc, input=label_name)
        # 显示标注的帧
        cv2.imshow("annotated_frame", annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):    # 按q键退出
            break
        if obj_img is not None:
            for i in range(len(obj_img)):
                print(f'第{i}组数据')
                dis, coordinate = camera.get_3d_camera_coordinate(obj_img[i][0])
                print(f'物体的四个顶点是{obj_img[i][1]}')
                print(f'物体的宽高是{obj_img[i][2]}')
                print(f'物体旋转的角度是{obj_img[i][3]}')
                print(f'相机坐标系下物体的x是{coordinate[0]}')
                print(f'相机坐标系下物体的y是{coordinate[1]}')
                print(f'相机坐标系下物体的z是{coordinate[2]}')
                # angle_radians = math.radians(obj_img[0][3])
                # angle_radians2 = math.radians(180)
                # # # angle_radians = obj_img[0][3] * math.pi / 180
                # cb_target_pose(arm, coordinate, obj_img[0][3])

                vertices = obj_img[i][1]
                # 从原始图像中截取目标框内的图像
                # 使用顶点数据计算最小外接矩形的坐标
                x_min = min(vertices[:, 0])
                x_max = max(vertices[:, 0])
                y_min = min(vertices[:, 1])
                y_max = max(vertices[:, 1])
                cropped_image = color_img[y_min:y_max, x_min:x_max]

                # 初始化 flag_pianjiao
                flag_pianjiao = 0

                # 转换为弧度
                if obj_img[i][2][0] < obj_img[i][2][1]:
                    angle_radians = math.radians(obj_img[i][3] - 35 + flag_pianjiao)  # 转换为弧度
                if obj_img[i][2][0] > obj_img[i][2][1]:
                    angle_radians = math.radians(obj_img[i][3] - 35 - 90 + flag_pianjiao)  # 转换为弧度
                # 将旋转角度转换为四元数
                quaternion = tf.transformations.quaternion_from_euler(0, 0, angle_radians)
        if flag_play == 0:
            #set play
            flag_play = 1
            play_start()    # 开始打开音频（仅一次）


if __name__ == '__main__':
    pt = 1
    conf = 0.10
    nc = 2
    input = "Coke_Drk"
    # input = "Dydo_Red_Tea"
    # input = "Tp"
    # input = "Hao_Li_You"
    # input = "Sunfl"
    # input = "Haoliyou_choco"
    # input = "water_Nongfu"
    # input = "Mo_Eg"
    # input = "shen"
    main(pt, conf, nc)
