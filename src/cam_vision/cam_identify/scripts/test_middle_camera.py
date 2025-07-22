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
import time    # 时间函数
import subprocess    # 连接输入/输出/错误管道
from cv_bridge import CvBridge, CvBridgeError
import numpy as np    # 数据处理模块
import threading
from typing import Union, Literal

import tf
import tf.transformations    # tf坐标处理模块
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String    # ROS的String消息类型
from cam_msgs.srv import HeadResult, HeadResultRequest, HeadResultResponse
from cam_msgs.srv import HandResult, HandResultRequest, HandResultResponse
from d435_ros.msg import RealsenseImage
from d435_ros.srv import PixelToCoordinate, PixelToCoordinateRequest

import custom_import    # 自定义引用模块
# import pyrealsense2 as rs
# from camera import realsense    # D435相机驱动模块


class Material_Identify:

    def __init__(self):
        
        # 实例化模型名称
        yolo_config = rospy.get_param("~yolo_config", "Body_Handly_Robot.pt")
        solver_config = rospy.get_param("~solver_config", "CDNet.pth")
        self.tag = rospy.get_param("~tag", True)
        self.conf = rospy.get_param("~conf", 0.3)
        self.nc = rospy.get_param("~nc", 2)

        yolo_weights_path = custom_import.PT_PAYH + '/files_pth/' + yolo_config
        solver_weights_path = custom_import.PT_PAYH + '/files_pth/' + solver_config
        
        # 实例化ROS通讯对象
        self.head_detect = rospy.Service("/head_detect_result", HeadResult, self._head_target_callback)

        self.sub_camera = rospy.Subscriber("/camera/image_raw", RealsenseImage, self._sub_image_callback, queue_size=1)
        # self.coordinate_client = rospy.ServiceProxy("/pixel_to_coordinate", PixelToCoordinate)
        
        # 初始化图像数据
        self.depth_image = np.array([])
        self.color_image = np.array([])

        self.image_ready = threading.Event()
        self.target_image = None
        # 初始化相机内参
        self.camera_matrix = np.array([])
        self.dist_coeffs = np.array([])

        # 初始化坐标数据
        self.camera_frame_id = ""

        # 初始化ar码识别参数
        # 设置 ArUco 字典和检测器参数
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
        self.parameters = cv2.aruco.DetectorParameters()
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX  # 更精确的角点检测
        self.parameters.cornerRefinementWinSize = 5
        self.parameters.cornerRefinementMaxIterations = 30
        self.camera_is_closed = True
        self.identify_thread_closed = True

        # 设置线程变量
        self._thread_identify = None

        # ... 其他初始化代码 ...
        self.angle_history = []  # 存储历史角度数据
        self.history_size = 50    # 考虑前100帧数据
        self.stability_threshold = 0.1  # 稳定性阈值(弧度)，可根据需要调整
        self.stable_status = False  # 当前是否稳定

        self.prev_rvec = None   # 前一帧数据
        self.smoothing_factor = 0.9  # 平滑系数，可调整

        # 初始化成功
        rospy.loginfo("Identification can begin!")

    def _smooth_angle(self, current_angle):
        # 初始化卡尔曼滤波器或低通滤波器
        prev_angle = 0
        alpha = 0.2  # 滤波系数，越小越平滑

        smoothed = alpha * current_angle + (1 - alpha) * prev_angle
        prev_angle = smoothed
        return smoothed


    def _thread_identify_function(self, aruco_num: int=582):
        
        while not self.identify_thread_closed:
            # 判断图像数据是否存在
            if not self.depth_image.any() or not self.color_image.any():
                time.sleep(0.1)
                continue

            # 相机未被关闭
            self.camera_is_closed = False

            # 转换图像格式
            display_image = cv2.cvtColor(self.color_image, cv2.COLOR_RGB2BGR)  # 转换为BGR格式用于显示
            gray_img = cv2.cvtColor(self.color_image, cv2.COLOR_RGB2GRAY)
            rospy.loginfo(f"-------->{self.camera_frame_id}<-----------")

            # 检测 ArUco 码
            corners, ids, _ = cv2.aruco.detectMarkers(gray_img, self.aruco_dict, parameters=self.parameters)

            # 可视化文本参数
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.6
            font_thickness = 2
            text_color = (0, 255, 0)  # 绿色
            
            # 修改检测部分的代码
            if ids is not None:
                # 绘制所有检测到的标记
                cv2.aruco.drawDetectedMarkers(display_image, corners, ids)
                
                # 在图像上显示检测到的ID数量
                cv2.putText(display_image, f"Detected {len(ids)} markers", 
                            (10, 30), font, font_scale, text_color, font_thickness)

                index = None
                if aruco_num in ids[:, 0]:
                    index = np.where(ids[:, 0] == aruco_num)[0][0]
                
                if index is not None:
                    # 计算 ArUco 码的位姿
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners[index], 0.02975, self.camera_matrix, self.dist_coeffs)
                    
                    # 确保rvec和tvec是正确形状 (3,1)
                    rvec = rvec[0].reshape(3, 1)
                    tvec = tvec[0].reshape(3, 1)

                    # 获取当前角度(使用旋转向量的范数)
                    current_angle = np.linalg.norm(rvec)

                    # 更新历史数据
                    self.angle_history.append(current_angle)
                    if len(self.angle_history) > self.history_size:
                        self.angle_history.pop(0)

                    # 检查稳定性
                    if len(self.angle_history) == self.history_size:
                        # 计算当前角度与前n帧的最大差异
                        max_diff = max(abs(current_angle - angle) for angle in self.angle_history[:-1])
                        self.stable_status = max_diff < self.stability_threshold

                    # 输出
                    print(f"Result=>: {self.stable_status}")
                    
                    # 角度平滑处理
                    if self.prev_rvec is None:
                        # 第一次检测，直接使用当前值
                        self.prev_rvec = rvec.copy()
                    else:
                        # 应用平滑滤波
                        rvec = self.smoothing_factor * rvec + (1 - self.smoothing_factor) * self.prev_rvec
                        self.prev_rvec = rvec.copy()
                    
                    # 绘制目标标记的坐标系
                    cv2.drawFrameAxes(display_image, self.camera_matrix, self.dist_coeffs,
                                    rvec, tvec, 0.05)
                    
                    # 在标记旁边显示位置信息
                    pos_text = f"ID {aruco_num}: Pos: {tvec.flatten().round(3)}"
                    cv2.putText(display_image, pos_text, 
                                (10, 60), font, font_scale, text_color, font_thickness)
                    
                    # 显示旋转信息和稳定性状态
                    status_color = (0, 255, 0) if self.stable_status else (0, 0, 255)
                    status_text = "Stable" if self.stable_status else "Unstable"
                    rot_text = f"Rot: {rvec.flatten().round(3)} | {status_text}"
                    cv2.putText(display_image, rot_text, 
                                (10, 90), font, font_scale, status_color, font_thickness)
                    
                    # 在图像上绘制稳定性指示器
                    cv2.circle(display_image, (display_image.shape[1] - 30, 30), 
                            10, status_color, -1)
                    
                    # 显示旋转信息
                    rot_text = f"Rot: {rvec.flatten().round(3)}"
                    cv2.putText(display_image, rot_text, 
                                (10, 90), font, font_scale, text_color, font_thickness)

                    # 将旋转向量转换为四元数
                    rotation_matrix, _ = cv2.Rodrigues(rvec)
                    quaternion = tf.transformations.quaternion_from_matrix(
                        np.vstack([
                            np.hstack([rotation_matrix, np.zeros((3,1))]),
                            np.array([0, 0, 0, 1])
                        ])
                    )
                    
                    # ... 其余代码保持不变 ...
                    
            else:
                # 没有检测到任何标记时显示提示
                cv2.putText(display_image, "No ArUco markers detected", 
                        (10, 30), font, font_scale, (0, 0, 255), font_thickness)

            # 显示处理后的图像
            cv2.imshow("ArUco Marker Detection", display_image)
            cv2.waitKey(1)  # 需要这个来更新显示窗口

            # 检查是否按下'q'键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.camera_is_closed = True
                self.identify_thread_closed = True
                break

        # 清理资源
        cv2.destroyAllWindows()
        self.depth_image = np.array([])
        self.color_image = np.array([])

    def _head_target_callback(self, req: HeadResultRequest):
        res = HeadResultResponse()
        time.sleep(0.1)
        
        # 识别
        if req.num == 582 and self.identify_thread_closed:
            self.identify_thread_closed = False
            self._thread_identify = threading.Thread(target=self._thread_identify_function, args=(req.num, ))
            self._thread_identify.start()
        elif req.num == 0 and self._thread_identify is not None:
            self.identify_thread_closed = True
            self._thread_identify.join()
        else:
            pass

        return res

    # 子线程中打开相机可视化界面
    def _display_image(self, window_name, fps: float=30.0):
        try:
            cv2.destroyAllWindows()
        except Exception as err:
            rospy.loginfo(err)

        cv2.namedWindow(window_name)
        while not rospy.is_shutdown() and not self.camera_is_closed:
            self.image_ready.wait()  # 等待图像数据准备好
            if self.target_image is not None:
                try:
                    cv2.imshow(window_name, self.target_image)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                    else:
                        pass
                except Exception as err:
                    rospy.loginfo(f"Error displaying image: { err }")
            self.image_ready.clear()  # 重置事件

        cv2.destroyAllWindows()

    
    # 定义四元数乘法函数
    def quaternion_multiply(self, q1, q2):
        """
        四元数乘法
        :param q1: 第一个四元数 (w + xi + yj + zk)
        :param q2: 第二个四元数 (w + xi + yj + zk)
        :return: 两个四元数的乘积
        """
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 + y1*w2 + z1*x2 - x1*z2
        z = w1*z2 + z1*w2 + x1*y2 - y1*x2
        return np.array([x, y, z, w])
    
    def publish_materiel_transform(self, tf_transform: TransformStamped):
        """
        发布一个物料识别坐标系变换。
        """
        # 以10Hz的频率运行，持续5秒
        rate = rospy.Rate(10)  # 10Hz
        end_time = rospy.Time.now() + rospy.Duration(2)  # 5秒后的时间戳
        broadcaster = tf.TransformBroadcaster()
        while rospy.Time.now() < end_time:
            # 发送变换
            broadcaster.sendTransform(
            (tf_transform.transform.translation.x,
             tf_transform.transform.translation.y,
             tf_transform.transform.translation.z),
            (tf_transform.transform.rotation.x,
             tf_transform.transform.rotation.y,
             tf_transform.transform.rotation.z,
             tf_transform.transform.rotation.w),
            rospy.Time.now(),
            tf_transform.child_frame_id,
            tf_transform.header.frame_id)
            rate.sleep()

    def publish_aruco_transform(self, tf_transform: TransformStamped):
        """
        发布一个物料识别坐标系变换。
        """
        # 以10Hz的频率运行，持续5秒
        rate = rospy.Rate(10)  # 10Hz
        broadcaster = tf.TransformBroadcaster()
        # 获取时间
        now_time = rospy.Time.now()
        
        # 发送左臂变换
        broadcaster.sendTransform(
        (tf_transform.transform.translation.x,
            tf_transform.transform.translation.y,
            tf_transform.transform.translation.z),
        (tf_transform.transform.rotation.x,
            tf_transform.transform.rotation.y,
            tf_transform.transform.rotation.z,
            tf_transform.transform.rotation.w),
        now_time,
        tf_transform.child_frame_id + "_left",
        tf_transform.header.frame_id + "_left")

        # 发送右臂变换
        broadcaster.sendTransform(
        (tf_transform.transform.translation.x,
            tf_transform.transform.translation.y,
            tf_transform.transform.translation.z),
        (tf_transform.transform.rotation.x,
            tf_transform.transform.rotation.y,
            tf_transform.transform.rotation.z,
            tf_transform.transform.rotation.w),
        now_time,
        tf_transform.child_frame_id + "_right",
        tf_transform.header.frame_id + "_right")
        
        # 等待函数
        rate.sleep()

    def _sub_image_callback(self, msg: RealsenseImage):
        bridge = CvBridge()
        try:
            # 获取图片数据
            deep_image = bridge.imgmsg_to_cv2(msg.deep, "16UC1")
            rgb_image = bridge.imgmsg_to_cv2(msg.rgb, "bgr8")
            rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
            self.depth_image = np.array(deep_image)
            self.color_image = np.array(rgb_image)
            self.camera_frame_id = msg.rgb.header.frame_id
            # 获取相机内参
            self.camera_matrix = np.array(msg.camera_info.K).reshape(3, 3).astype(np.float32)
            self.dist_coeffs = np.array(msg.camera_info.D).astype(np.float32)
        except CvBridgeError as err:
            self.depth_image = np.array([])
            self.color_image = np.array([])
            self.camera_matrix = np.array([])
            self.dist_coeffs = np.array([])
            rospy.loginfo(err)
        pass


if __name__ == '__main__':
    rospy.init_node("material_detect_node")
    action = Material_Identify()


    rospy.spin()
