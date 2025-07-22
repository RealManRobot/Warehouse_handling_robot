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
from cam_Inference.solver import *    # 对相机进行计算
from cam_Inference.yolo_inference import Detect    # 识别模块


class Material_Identify:

    def __init__(self):
        # 实例化识别器
        self.detect = Detect()
        # 实例化图片计算器
        self.solver = Solver()
        
        # 实例化模型名称
        yolo_config = rospy.get_param("~yolo_config", "Body_Handly_Robot.pt")
        solver_config = rospy.get_param("~solver_config", "CDNet.pth")
        self.tag = rospy.get_param("~tag", True)
        self.conf = rospy.get_param("~conf", 0.3)
        self.nc = rospy.get_param("~nc", 2)

        yolo_weights_path = custom_import.PT_PAYH + '/files_pth/' + yolo_config
        solver_weights_path = custom_import.PT_PAYH + '/files_pth/' + solver_config

        self.model, self.solver_weights = self.detect.gen_model(tag=True, yolo_weights=yolo_weights_path, solver=self.solver,
                                             solver_weights_path=solver_weights_path)
        
        # 实例化ROS通讯对象
        self.hand_detect = rospy.Service("/hand_detect_result", HandResult, self._hand_target_callback)
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
        self.camera_is_closed = True

        # 头部相机识别aruco码
        self.angle_history = []  # 存储历史角度数据
        self.history_size = 10    # 考虑前10帧数据
        self.stability_threshold = 0.1  # 稳定性阈值(弧度)，可根据需要调整
        self.stable_status = False  # 当前是否稳定

        self.prev_rvec = None   # 前一帧数据
        self.smoothing_factor = 0.8  # 平滑系数，可调整

        # 初始化成功
        rospy.loginfo("Identification can begin!")

    # 通过像素坐标->三维空间位置
    def __get_camera_coords(self, depth_image, camera_matrix, u, v):
        if depth_image is None or camera_matrix is None:
            rospy.logwarn("Depth image or camera info not yet received.")
            return [0.0, 0.0, 0.0]

        # 获取像素点深度值
        depth = depth_image[v, u] / 1000.0  # 转换为米

        # 获取相机内参
        fx = float(camera_matrix[0, 0])
        fy = float(camera_matrix[1, 1])
        cx = float(camera_matrix[0, 2])
        cy = float(camera_matrix[1, 2])

        # 坐标转换
        X = (float(u) - cx) * depth / fx
        Y = (float(v) - cy) * depth / fy
        Z = depth

        return [X, Y, Z]
    
    def __get_average_depth(self, depth_image, x, y, ranged: int=3):

        # 循环获取平均值
        number_depth = 0
        number_depth_sum = 0

        for u in range(x - int(ranged / 2), x + int(ranged / 2) + 1):
            for v in range(y - int(ranged / 2), y + int(ranged / 2) + 1):
                # 求出深度值
                coordinate = self.__get_camera_coords(depth_image, self.camera_matrix, u, v)
                # 判断深度值是否为零值
                if coordinate[2] != 0:
                    number_depth_sum += coordinate[2]
                    number_depth += 1

        # 求得平均值
        if not number_depth:
            return 0.0, 0.0, 0.0
        else:
            return coordinate[0], coordinate[1], number_depth_sum / float(number_depth)

    def _hand_target_callback(self, req: HandResultRequest):
        res = HandResultResponse()
        time_begin = rospy.get_rostime()
        time_end = rospy.get_rostime()
        time.sleep(0.1)

        # 开始识别
        while not rospy.is_shutdown() and (time_end - time_begin).to_sec() < req.delayed:
            
            # 判断图像数据是否存在
            if not self.depth_image.any() or not self.color_image.any() or self.camera_frame_id != req.link_name:
                rospy.sleep(0.1)
                # 记录当前时间
                time_end = rospy.get_rostime()
                continue
            
            # 相机未被关闭
            self.camera_is_closed = False

            # 处理划分图像数据
            color_img = cv2.cvtColor(self.color_image, cv2.COLOR_RGB2BGR)
            depth_img = self.depth_image.copy()
            # gray_img = cv2.cvtColor(self.color_image, cv2.COLOR_RGB2GRAY)
            rospy.loginfo(f"-------->{ self.camera_frame_id }<-----------")

            # 识别
            deep_data_depth_esi = self.solver.test(img=color_img)
            deep_data3 = cv2.cvtColor(deep_data_depth_esi, cv2.COLOR_GRAY2BGR)
            if self.tag:
                results = self.model(color_img, deep_data3, conf=self.conf)    # 使用pt模型进行推理

                annotated_frame, obj_img = self.detect.backward_handle_output(self.tag, results, color_img, depth_img, 
                                                                              self.solver_weights, nc=self.nc, input=req.label)


                print("obj_img=", obj_img)

                if obj_img is not None and len(obj_img):

                    # ————————————————————————————————————————————————————————————————————————————————————
                    # 初始化最小 pose_z 和对应的索引
                    min_pose_z = float('inf')  # 初始化为正无穷
                    min_pose_z_index = -1
                    # 遍历 obj_img 列表
                    for i in range(len(obj_img)):
                        res_coordinate = self.__get_average_depth(depth_img, obj_img[i][0][0], obj_img[i][0][1], 3)
                        # res_coordinate = self.__get_camera_coords(depth_img, self.camera_matrix, obj_img[i][0][0], obj_img[i][0][1])

                        # 检查当前 pose_z 是否小于当前最小值且不为零
                        if res_coordinate[2] != 0.0 and res_coordinate[2] < min_pose_z:
                            min_pose_z = res_coordinate[2]
                            min_pose_z_index = i

                    print("x, y=", obj_img[min_pose_z_index][0][0], obj_img[min_pose_z_index][0][1])

                    # 输出最小 pose_z 及其对应的索引
                    if min_pose_z_index != -1:

                        print(f'The {min_pose_z_index} group of data')
                        # res_coordinate = self.coordinate_client(PixelToCoordinateRequest(obj_img[min_pose_z_index][0][0], 
                        #                                                                  obj_img[min_pose_z_index][0][1]))

                        # 得到z轴最小值的目标框相关信息数据
                        res_coordinate = self.__get_average_depth(depth_img, obj_img[min_pose_z_index][0][0], obj_img[min_pose_z_index][0][1])

                        print(f'The four vertices of the object are: {obj_img[min_pose_z_index][1]}')
                        print(f'The width and height of the object are: {obj_img[min_pose_z_index][2]}')
                        print(f'The angle of rotation of an object is: {obj_img[min_pose_z_index][3]}')
                        print(f'The [x] of an object in the camera coordinate system is: {res_coordinate[0]}')
                        print(f'The [y] of an object in the camera coordinate system is: {res_coordinate[1]}')
                        print(f'The [z] of an object in the camera coordinate system is: {res_coordinate[2]}')

                        # 转化为宽高
                        if req.label == "AnNiu":
                            angle_radians = self.__get_grab_angle(*(obj_img[min_pose_z_index][2]), 
                                                                  obj_img[min_pose_z_index][3], 
                                                                  "width")
                        else:
                            angle_radians = self.__get_grab_angle(*(obj_img[min_pose_z_index][2]), 
                                                                  obj_img[min_pose_z_index][3], 
                                                                  "height")

                        # 将旋转角度转换为四元数
                        quaternion = tf.transformations.quaternion_from_euler(0, 0, angle_radians)
                        
                        # 发布识别结果坐标系
                        tf_transform = TransformStamped()

                        tf_transform.header.frame_id = self.camera_frame_id
                        tf_transform.child_frame_id = "target_" + self.camera_frame_id
                        tf_transform.header.stamp = rospy.Time.now()
                        tf_transform.transform.translation.x = res_coordinate[0]
                        tf_transform.transform.translation.y = res_coordinate[1]
                        tf_transform.transform.translation.z = res_coordinate[2]
                        tf_transform.transform.rotation.x = quaternion[0]
                        tf_transform.transform.rotation.y = quaternion[1]
                        tf_transform.transform.rotation.z = quaternion[2]
                        tf_transform.transform.rotation.w = quaternion[3]
                        # rospy.loginfo(tf_transform)
                        # 提取必要的信息并广播变换
                        tf_thread = threading.Thread(target=self.publish_materiel_transform, args=(tf_transform,))
                        tf_thread.start()
                        time.sleep(0.5)

                        # 监听target对于base的相对关系
                        tf_listener = tf.TransformListener()
                        base_frame_id = self.camera_frame_id.split("_")[-1] + "_base"
                        try:
                            tuple_frame_id = (base_frame_id, tf_transform.child_frame_id)
                            # 等待tf树中的数据变得可用
                            tf_listener.waitForTransform(*tuple_frame_id, rospy.Time(0), rospy.Duration(1.0))
                            print(base_frame_id, "---->", tf_transform.child_frame_id)
                            (trans, rot) = tf_listener.lookupTransform(*tuple_frame_id, rospy.Time(0))
                            # 姿态角调整
                            eulers = tf.transformations.euler_from_quaternion(rot)
                            rospy.loginfo("tf----->:%f, %f, %f", *eulers)
                            # 得到坐标转换关系
                            res.header.frame_id = base_frame_id
                            res.header.stamp = rospy.Time.now()
                            res.angle = angle_radians
                            res.pose.position.x = trans[0]
                            res.pose.position.y = trans[1]
                            res.pose.position.z = trans[2]
                            # 只计算航偏角
                            res.pose.orientation.x, res.pose.orientation.y, res.pose.orientation.z, res.pose.orientation.w =\
                            tf.transformations.quaternion_from_euler(*eulers)
                            rospy.loginfo("Transform: %s %s", trans, rot)
                        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as err:
                            rospy.loginfo(str(err))
                            tf_thread.join()
                            continue
                        
                        # cv2.destroyAllWindows()
                        # 关闭相机
                        self.camera_is_closed = True
                        # 清空照片, 等待新数据响应
                        self.depth_image = np.array([])
                        self.color_image = np.array([])
                        return res

                    else:
                        print('No data found with pose_z non-zero!')
                        continue

            # 得到当前时间
            time_end = rospy.get_rostime()
        
        # 关闭相机
        self.camera_is_closed = True
        # 清空照片, 防止下次误识别
        self.depth_image = np.array([])
        self.color_image = np.array([])
        # self.camera_show_thread.join()
        # 识别超时
        return res


    def _head_target_callback(self, req: HeadResultRequest):
        res = HeadResultResponse()
        time_begin = rospy.get_rostime()
        time_end = rospy.get_rostime()
        time.sleep(0.1)

        # 开始识别之前将相机数据晴空，等待下一帧
        self.camera_is_closed = True
        self.depth_image = np.array([])
        self.color_image = np.array([])
        # 开始识别
        while not rospy.is_shutdown() and (time_end - time_begin).to_sec() < req.delayed:
            # 判断图像数据是否存在
            if not self.depth_image.any() or not self.color_image.any() or self.camera_frame_id != req.link_name:
                # 记录当前时间
                time_end = rospy.get_rostime()
                time.sleep(0.1)
                continue

            # 相机未被关闭
            self.camera_is_closed = False

            # 处理划分图像数据
            # color_img = cv2.cvtColor(self.color_image, cv2.COLOR_RGB2BGR)
            gray_img = cv2.cvtColor(self.color_image, cv2.COLOR_RGB2GRAY)
            rospy.loginfo(f"-------->{ self.camera_frame_id }<-----------")

            # 检测 ArUco 码
            corners, ids, _ = cv2.aruco.detectMarkers(gray_img, self.aruco_dict, parameters=self.parameters)

            # 如果检测到 ArUco 码
            if ids is not None:
                index = None
                if req.num in ids[:, 0]:
                    index = np.where(ids[:, 0] == req.num)[0][0]
                
                if index is not None:
                    # 计算 ArUco 码的位姿
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners[index], 0.02975, self.camera_matrix, self.dist_coeffs)
                    
                    # 确保rvec和tvec是正确形状 (3,1)
                    rvec_ = rvec[0].reshape(3, 1)

                    # 获取当前角度(使用旋转向量的范数)
                    current_angle = np.linalg.norm(rvec_)

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
                        self.prev_rvec = rvec_.copy()
                    else:
                        # 应用平滑滤波
                        rvec_ = self.smoothing_factor * rvec_ + (1 - self.smoothing_factor) * self.prev_rvec
                        self.prev_rvec = rvec_.copy()
                    
                    if not self.stable_status:
                         # 打印三维坐标和姿态（旋转向量）
                        print(f"Marker ID: {ids[index][0]}, Position: {tvec[0][0]}, Rotation Vector: {rvec_}")
                        self.depth_image = np.array([])
                        self.color_image = np.array([])
                        continue

                    # 将旋转角度转换为四元数
                    quaternion = tf.transformations.quaternion_from_euler(rvec_[0], rvec_[2], -rvec_[1])

                    # 得到坐标转换关系,矫正坐标系与工作坐标系对应
                    # 得到识别到的坐标系基于夹爪坐标系的旋转四元数算子
                    # 绕x轴旋转90度的四元数的算子
                    q_x180 = np.array([1, 0, 0, 0])
                    # 单位变换/不进行改变
                    q_y180 = np.array([0, 0, 0, 1])
                    # 先绕x轴旋转，再绕z轴旋转
                    q_rotated = self.quaternion_multiply(self.quaternion_multiply(quaternion, q_x180), q_y180)
                    
                    # 发布识别结果坐标系
                    tf_transform = TransformStamped()

                    tf_transform.header.frame_id = self.camera_frame_id
                    tf_transform.child_frame_id = "target_" + self.camera_frame_id
                    tf_transform.header.stamp = rospy.Time.now()
                    tf_transform.transform.translation.x = tvec[0][0][0]
                    tf_transform.transform.translation.y = tvec[0][0][1]
                    tf_transform.transform.translation.z = tvec[0][0][2]
                    tf_transform.transform.rotation.x = q_rotated[0]
                    tf_transform.transform.rotation.y = q_rotated[1]
                    tf_transform.transform.rotation.z = q_rotated[2]
                    tf_transform.transform.rotation.w = q_rotated[3]
                    # rospy.loginfo(tf_transform)
                    # 提取必要的信息并广播变换
                    tf_thread = threading.Thread(target=self.publish_aruco_transform, args=(tf_transform,))
                    tf_thread.start()
                    time.sleep(0.5)
    
                    # 监听target对于base的相对关系
                    tf_listener = tf.TransformListener()
                    # 双臂关系
                    bearing = ["left", "right"]

                    # 反馈识别结果 
                    res.header.frame_id = "middel_base"
                    res.header.stamp = rospy.Time.now()

                    # 双臂循环
                    for index in range(len(res.pose)):
                        child_frame_id = tf_transform.child_frame_id + "_" + bearing[index]
                        base_frame_id = bearing[index] + "_" + "base"
                        try:
                            tuple_frame_id = (base_frame_id, child_frame_id)
                            # 等待tf树中的数据变得可用
                            tf_listener.waitForTransform(*tuple_frame_id, rospy.Time(0), rospy.Duration(1.0))
                            print(base_frame_id, "---->", child_frame_id)
                            (trans, rot) = tf_listener.lookupTransform(*tuple_frame_id, rospy.Time(0))
                            
                            # 姿态角调整
                            eulers = tf.transformations.euler_from_quaternion(rot)
                            rospy.loginfo("tf----->:%f, %f, %f", *eulers)
                            res.pose[index].position.x = trans[0]
                            res.pose[index].position.y = trans[1]
                            res.pose[index].position.z = trans[2]

                            # 只计算航偏角
                            res.pose[index].orientation.x, res.pose[index].orientation.y, res.pose[index].orientation.z, res.pose[index].orientation.w =\
                            tf.transformations.quaternion_from_euler(*eulers)
                            
                            rospy.loginfo("Transform: %s %s", trans, q_rotated)
                        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as err:
                            rospy.loginfo("err: %s", err)
                            tf_thread.join()
                            continue

                    # 关闭相机
                    self.camera_is_closed = True
                    self.stable_status = False
                    self.depth_image = np.array([])
                    self.color_image = np.array([])
                    self.angle_history = []
                    # self.camera_show_thread.join()
                    # cv2.destroyAllWindows()
                    return res
                else:
                    pass
            else:
                pass

            time_end = rospy.get_rostime()
        # 关闭相机
        self.camera_is_closed = True
        # 清空照片, 防止下次误识别
        self.depth_image = np.array([])
        self.color_image = np.array([])
        # self.camera_show_thread.join()
        # 识别超时
        return res

    
    # 通过传递mask最小外接矩形角度依赖类型来判断旋转角度
    def __get_grab_angle(self, width, height, angle, type: Union[str, Literal["width", "height"]] = "width"):
        if type == "width":
            # 通过mask计算识别角度, 弧度值
            if width < height:
                angle = -(90 - angle)
            else:
                angle = angle

            angle_radians = math.pi * (angle / 180)
        elif type == "height":
            # 通过mask计算识别角度, 弧度值
            if width > height:
                angle = -(90 - angle)
            else:
                angle = angle

            angle_radians = math.pi * (angle / 180)
        else:
            angle_radians = math.pi * (angle / 180)

        print("End Get Angle is %d", angle_radians)

        return angle_radians

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
        end_time = rospy.Time.now() + rospy.Duration(2)  # 5秒后的时间戳
        broadcaster = tf.TransformBroadcaster()
        while rospy.Time.now() < end_time:
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