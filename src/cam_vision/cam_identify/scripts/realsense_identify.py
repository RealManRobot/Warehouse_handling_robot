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
import subprocess    # 连接输入/输出/错误管道
import time    # 事件处理模块
from cv_bridge import CvBridge, CvBridgeError
import numpy as np    # 数据处理模块
import threading
from typing import Union, Literal

import tf
import tf.transformations    # tf坐标处理模块
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String    # ROS的String消息类型
from cam_msgs.srv import DetectResult, DetectResultRequest, DetectResultResponse   # 获取识别结果的服务消息类型
# from d435_ros.msg import RealsenseImage
# from d435_ros.srv import PixelToCoordinate, PixelToCoordinateRequest
from sensor_msgs.msg import Image as Image_
from sensor_msgs.msg import CameraInfo

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

        # 设置相机数据模板
        camera = {
                "color": None,
                "depth": None,
                "info": {
                    "camera_matrix": None,
                    "dist_coeffs": None
                }
            }
        
        self.camera_left = camera.copy()
        self.camera_middle = camera.copy()
        self.camera_right = camera.copy()
        
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
        self.server_detect = rospy.Service("/get_detect_result", DetectResult, self._target_item_callback)
        # self.sub_camera = rospy.Subscriber("/camera/image_raw", RealsenseImage, self._sub_image_callback, queue_size=1)
        # self.coordinate_client = rospy.ServiceProxy("/pixel_to_coordinate", PixelToCoordinate)

        self.camera_left_color = rospy.Subscriber("/camera_left/color/image_raw", Image_, self.__sub_left_color_callback, queue_size=10)
        self.camera_maddile_color = rospy.Subscriber("/camera_middle/color/image_raw", Image_, self.__sub_middle_color_callback, queue_size=10)
        self.camera_right_color = rospy.Subscriber("/camera_right/color/image_raw", Image_, self.__sub_right_color_callback, queue_size=10)

        self.camera_left_depth = rospy.Subscriber("/camera_left/aligned_depth_to_color/image_raw", Image_, self.__sub_left_depth_callback, queue_size=10)
        self.camera_middle_depth = rospy.Subscriber("/camera_middle/aligned_depth_to_color/image_raw", Image_, self.__sub_middle_depth_callback, queue_size=10)
        self.camera_right_depth = rospy.Subscriber("/camera_right/aligned_depth_to_color/image_raw", Image_, self.__sub_right_depth_callback, queue_size=10)
        
        self.camera_left_info = rospy.Subscriber("/camera_left/color/camera_info", CameraInfo, self.__sub_left_info_callback, queue_size=10)
        self.camera_maddile_info = rospy.Subscriber("/camera_middle/color/camera_info", CameraInfo, self.__sub_middle_info_callback, queue_size=10)
        self.camera_right_info = rospy.Subscriber("/camera_right/color/camera_info", CameraInfo, self.__sub_right_info_callback, queue_size=10)

        # 初始化坐标数据
        self.tf_transform = TransformStamped()

        # 初始化ar码识别参数
        # 设置 ArUco 字典和检测器参数
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
        self.parameters = cv2.aruco.DetectorParameters()
        # self.camera_is_closed = True

        # 初始化成功
        rospy.loginfo("Identification can begin!")

    # 更新数据
    def _update_image_data(self, link_name: str):
        if link_name == "camera_left":
            self.camera_left["color"] = None
            self.camera_left["depth"] = None
        elif link_name == "camera_middle":
            self.camera_middle["color"] = None
            self.camera_middle["depth"] = None
        elif link_name == "camera_right":
            self.camera_right["color"] = None
            self.camera_right["depth"] = None
        else:
            pass
    
    # rgb数据获取
    def __sub_left_color_callback(self, msg: Image_):
        bridge = CvBridge()
        try:
            # 获取图片数据
            rgb_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
            self.camera_left["color"] = np.array(rgb_image)
        except CvBridgeError as err:
            self.camera_left["color"] = None
            rospy.logerr(err)

    def __sub_middle_color_callback(self, msg: Image_):
        bridge = CvBridge()
        # rospy.loginfo("middle")
        try:
            # 获取图片数据
            rgb_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
            self.camera_middle["color"] = np.array(rgb_image)
        except CvBridgeError as err:
            self.camera_middle["color"] = None
            rospy.logerr(err)

    def __sub_right_color_callback(self, msg: Image_):
        bridge = CvBridge()
        try:
            # 获取图片数据
            rgb_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
            self.camera_right["color"] = np.array(rgb_image)
        except CvBridgeError as err:
            self.camera_right["color"] = None
            rospy.logerr(err)
    
    # 深度数据获取
    def __sub_left_depth_callback(self, msg: Image_):
        bridge = CvBridge()
        try:
            # 获取图片数据
            deep_image = bridge.imgmsg_to_cv2(msg, "16UC1")
            deep_image = cv2.cvtColor(deep_image, cv2.COLOR_BGR2RGB)
            self.camera_left["depth"] = np.array(deep_image)
        except CvBridgeError as err:
            self.camera_left["depth"] = None
            rospy.logerr(err)
    
    def __sub_middle_depth_callback(self, msg: Image_):
        bridge = CvBridge()
        try:
            # 获取图片数据
            deep_image = bridge.imgmsg_to_cv2(msg, "16UC1")
            deep_image = cv2.cvtColor(deep_image, cv2.COLOR_BGR2RGB)
            self.camera_middle["depth"] = np.array(deep_image)
        except CvBridgeError as err:
            self.camera_middle["depth"] = None
            rospy.logerr(err)

    def __sub_right_depth_callback(self, msg: Image_):
        bridge = CvBridge()
        try:
            # 获取图片数据
            deep_image = bridge.imgmsg_to_cv2(msg, "16UC1")
            deep_image = cv2.cvtColor(deep_image, cv2.COLOR_BGR2RGB)
            self.camera_right["depth"] = np.array(deep_image)
        except CvBridgeError as err:
            self.camera_right["depth"] = None
            rospy.logerr(err)

    # 相机内参获取
    def __sub_left_info_callback(self, msg: CameraInfo):
        self.camera_left["info"]["camera_matrix"] = np.array(msg.K).reshape(3, 3).astype(np.float32)
        self.camera_left["info"]["dist_coeffs"] = np.array(msg.D).astype(np.float32)
        self.camera_left_info.unregister()

    def __sub_middle_info_callback(self, msg: CameraInfo):
        self.camera_middle["info"]["camera_matrix"] = np.array(msg.K).reshape(3, 3).astype(np.float32)
        self.camera_middle["info"]["dist_coeffs"] = np.array(msg.D).astype(np.float32)
        self.camera_maddile_info.unregister()

    def __sub_right_info_callback(self, msg: CameraInfo):
        self.camera_right["info"]["camera_matrix"] = np.array(msg.K).reshape(3, 3).astype(np.float32)
        self.camera_right["info"]["dist_coeffs"] = np.array(msg.D).astype(np.float32)
        self.camera_right_info.unregister()

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

        return [X[0], Y[0], Z[0]]
    
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
    
    # 得到相机数据
    def get_camera_data(self, link_name: str):
        if link_name == "camera_left":
            return self.camera_left["color"], \
                self.camera_left["depth"], \
                    self.camera_left["info"]["camera_matrix"], \
                        self.camera_left["info"]["dist_coeffs"]
        elif link_name == "camera_middle":
            return self.camera_middle["color"], \
                self.camera_middle["depth"], \
                    self.camera_middle["info"]["camera_matrix"], \
                        self.camera_middle["info"]["dist_coeffs"]
        elif link_name == "camera_right":
            return self.camera_right["color"], \
                self.camera_right["depth"], \
                    self.camera_right["info"]["camera_matrix"], \
                        self.camera_right["info"]["dist_coeffs"]
        else:
            return [None] * 4

    # 服务回调函数
    def _target_item_callback(self, req: DetectResultRequest):
        res = DetectResultResponse()
        time_begin = rospy.get_rostime()
        time_end = rospy.get_rostime()
        time.sleep(1)
        # 得到请求的坐标系是？
        camera_frame_id = req.link_name
        while not rospy.is_shutdown() and (time_end - time_begin).to_sec() <= req.delayed:
            color_image, depth_image, camera_matrix, dist_coeffs = self.get_camera_data(req.link_name)
            
            # 记录当前时间
            time_end = rospy.get_rostime()
            # 判断图像数据是否存在
            if depth_image is None or color_image is None:
                rospy.sleep(0.1)
                rospy.logwarn(f"{ camera_frame_id }-image is NULL...")
                continue
            
            # 相机未被关闭
            # self.camera_is_closed = False TODO

            # 处理划分图像数据
            color_img = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)    # 颜色空间转换
            # deep_data3 = color_img

            gray_img = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

            rospy.logerr(f"-------->{ camera_frame_id }<-----------")
            # 判断识别类型
            if camera_frame_id == "camera_left" or camera_frame_id == "camera_right":
                deep_data_depth_esi = self.solver.test(img=color_img)
                deep_data3 = cv2.cvtColor(deep_data_depth_esi, cv2.COLOR_GRAY2BGR)
                if self.tag:
                    results = self.model(color_img, deep_data3, conf=self.conf)    # 使用pt模型进行推理

                    annotated_frame, obj_img = self.detect.backward_handle_output(self.tag, results, color_img, depth_image,
                                                                            self.solver_weights, nc=self.nc, input=req.label)
                
                # 设置子线程
                # self.camera_show_thread = threading.Thread(target=self._display_image, 
                #                                         args=(camera_frame_id, 5))
                # self.camera_show_thread.start()
                # 显示标注的帧
                # cv2.imshow("annotated_frame", annotated_frame)
                # self.target_image = annotated_frame  TODO
                # self.image_ready.set()  # 通知子线程图像数据已准备好 TODO
                # [-0.37246005846664193, -0.5093683099479824, -0.15609751899369853] [0.04299721283834015, -0.999060366928816, 0.0008333952864762921, 0.0053785102482589876]
                # [-0.39507033961835186, -0.4667970777652482, -0.15493685205716728] [-0.04450651637784724, -0.9989942440043784, 0.0013208628490200013, 0.00528448364193877]
                if obj_img is not None:
                    for i in range(len(obj_img)):
                        print(f'第{i}组数据')
                        res_coordinate = self.__get_camera_coords(depth_image, camera_matrix, obj_img[i][0][0], obj_img[i][0][1])
                        # dis, coordinate = camera.get_3d_camera_coordinate(obj_img[i][0])
                        print(f'物体的四个顶点是{obj_img[i][1]}')
                        print(f'物体的宽高是{obj_img[i][2]}')
                        print(f'物体旋转的角度是{obj_img[i][3]}')
                        print(f'相机坐标系下物体的x是{res_coordinate[0]}')
                        print(f'相机坐标系下物体的y是{res_coordinate[1]}')
                        print(f'相机坐标系下物体的z是{res_coordinate[2]}')
                        
                        # 判断是否为零， 用np.all进行比较，直接用res_coordinate[0] == 0.0 and ...比较容易出问题
                        if np.all(res_coordinate == [0.0, 0.0, 0.0]):
                            continue
                        else:
                            pass

                        # 转化为宽高
                        if req.label == "AnNiu":
                            angle_radians = self.__get_grab_angle(*(obj_img[i][2]), obj_img[i][3], "width")
                        else:
                            angle_radians = self.__get_grab_angle(*(obj_img[i][2]), obj_img[i][3], "height")

                        # 将旋转角度转换为四元数
                        quaternion = tf.transformations.quaternion_from_euler(0, 0, angle_radians)

                        # 发布识别结果坐标系
                        self.tf_transform.header.frame_id = camera_frame_id
                        self.tf_transform.child_frame_id = "target_" + camera_frame_id
                        self.tf_transform.header.stamp = rospy.Time.now()
                        self.tf_transform.transform.translation.x = float(res_coordinate[0])
                        self.tf_transform.transform.translation.y = float(res_coordinate[1])
                        self.tf_transform.transform.translation.z = float(res_coordinate[2])
                        self.tf_transform.transform.rotation.x = float(quaternion[0])
                        self.tf_transform.transform.rotation.y = float(quaternion[1])
                        self.tf_transform.transform.rotation.z = float(quaternion[2])
                        self.tf_transform.transform.rotation.w = float(quaternion[3])
                        # rospy.loginfo(tf_transform)
                        # 提取必要的信息并广播变换
                        tf_thread = threading.Thread(target=self.publish_transform)
                        tf_thread.start()
        
                        # 监听target对于base的相对关系
                        tf_listener = tf.TransformListener()
                        base_frame_id = "base_link"
                        try:
                            tuple_frame_id = (base_frame_id, self.tf_transform.child_frame_id)
                            # 等待tf树中的数据变得可用
                            tf_listener.waitForTransform(*tuple_frame_id, rospy.Time(0), rospy.Duration(1.0))
                            print(base_frame_id, "---->", self.tf_transform.child_frame_id)
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
                            tf.transformations.quaternion_from_euler(3.1416, 0.0, eulers[2])
                            rospy.loginfo("Transform: %s %s", trans, rot)
                        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as err:
                            rospy.loginfo("err: %s", err)
                            tf_thread.join()
                            continue
                        # cv2.destroyAllWindows()
                        # 关闭相机
                        # self.camera_is_closed = True
                        # self.camera_show_thread.join()
                        # 刷新数据
                        self._update_image_data(camera_frame_id)
                        return res
            elif camera_frame_id == "camera_middle":
                # 检测 ArUco 码
                corners, ids, _ = cv2.aruco.detectMarkers(gray_img, self.aruco_dict, parameters=self.parameters)

                # 如果检测到 ArUco 码
                if ids is not None:
                    index = None
                    if 582 in ids[:, 0]:
                        index = np.where(ids[:, 0] == 582)[0][0]
                    
                    if index is not None:
                        # 计算 ArUco 码的位姿
                        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[index], 0.03, camera_matrix, dist_coeffs)

                        # 绘制标记和坐标轴
                        cv2.aruco.drawDetectedMarkers(color_image, corners, ids)
                        cv2.drawFrameAxes(color_image, camera_matrix, dist_coeffs, rvec[0], tvec[0], 0.1)
                        
                        # 将图片传为显示框
                        # self.target_image = color_image
                        # self.image_ready.set()  # 通知子线程图像数据已准备好

                        # 打印三维坐标和姿态（旋转向量）
                        print(f"Marker ID: {ids[index][0]}, Position: {tvec[0][0]}, Rotation Vector: {rvec[0][0]}")

                        # 将旋转角度转换为四元数
                        quaternion = tf.transformations.quaternion_from_euler(rvec[0][0][0], rvec[0][0][2], -rvec[0][0][1])

                        # 得到坐标转换关系,矫正坐标系与工作坐标系对应
                        # 得到识别到的坐标系基于夹爪坐标系的旋转四元数算子
                        # 绕x轴旋转90度的四元数的算子
                        q_x180 = np.array([1, 0, 0, 0])
                        # 单位变换/不进行改变
                        q_y180 = np.array([0, 0, 0, 1])
                        # 先绕x轴旋转，再绕z轴旋转
                        q_rotated = self.quaternion_multiply(self.quaternion_multiply(quaternion, q_x180), q_y180)
                        
                        # 发布识别结果坐标系
                        self.tf_transform.header.frame_id = camera_frame_id
                        self.tf_transform.child_frame_id = "target_" + camera_frame_id
                        self.tf_transform.header.stamp = rospy.Time.now()
                        self.tf_transform.transform.translation.x = tvec[0][0][0]
                        self.tf_transform.transform.translation.y = tvec[0][0][1]
                        self.tf_transform.transform.translation.z = tvec[0][0][2]
                        self.tf_transform.transform.rotation.x = q_rotated[0]
                        self.tf_transform.transform.rotation.y = q_rotated[1]
                        self.tf_transform.transform.rotation.z = q_rotated[2]
                        self.tf_transform.transform.rotation.w = q_rotated[3]
                        # rospy.loginfo(tf_transform)
                        # 提取必要的信息并广播变换
                        tf_thread = threading.Thread(target=self.publish_transform)
                        tf_thread.start()
        
                        # 监听target对于base的相对关系
                        tf_listener = tf.TransformListener()
                        base_frame_id = "base_link"
                        try:
                            tuple_frame_id = (base_frame_id, self.tf_transform.child_frame_id)
                            # 等待tf树中的数据变得可用
                            tf_listener.waitForTransform(*tuple_frame_id, rospy.Time(0), rospy.Duration(1.0))
                            print(base_frame_id, "---->", self.tf_transform.child_frame_id)
                            (trans, rot) = tf_listener.lookupTransform(*tuple_frame_id, rospy.Time(0))
                            
                            # 姿态角调整
                            eulers = tf.transformations.euler_from_quaternion(rot)
                            rospy.loginfo("tf----->:%f, %f, %f", *eulers)
                            # 反馈识别结果 
                            res.header.frame_id = base_frame_id
                            res.header.stamp = rospy.Time.now()
                            res.angle = eulers[2]
                            res.pose.position.x = trans[0]
                            res.pose.position.y = trans[1]
                            res.pose.position.z = trans[2]

                            # 只计算航偏角
                            res.pose.orientation.x, res.pose.orientation.y, res.pose.orientation.z, res.pose.orientation.w =\
                            tf.transformations.quaternion_from_euler(-1.57, 0.0, eulers[2])
                            # res.pose.orientation.x = rot[0]
                            # res.pose.orientation.y = rot[1]
                            # res.pose.orientation.z = rot[2]
                            # res.pose.orientation.w = rot[3]
                            
                            rospy.loginfo("Transform: %s %s", trans, q_rotated)
                        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as err:
                            rospy.loginfo("err: %s", err)
                            tf_thread.join()
                            continue

                        # self.camera_show_thread.join()
                        # cv2.destroyAllWindows()
                        # 刷新数据
                        self._update_image_data(camera_frame_id)
                        return res
                    else:
                        pass
                else:
                    pass

                # 刷新数据
                self._update_image_data(camera_frame_id)
            else:
                # 刷新数据
                self._update_image_data(camera_frame_id)
                return res
        
        # 识别超时
        return res
    
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
    
    def publish_transform(self):
        """
        发布一个坐标系变换。
        """
        # 以10Hz的频率运行，持续5秒
        rate = rospy.Rate(10)  # 10Hz
        end_time = rospy.Time.now() + rospy.Duration(2)  # 5秒后的时间戳
        broadcaster = tf.TransformBroadcaster()
        while rospy.Time.now() < end_time:
            # 发送变换
            broadcaster.sendTransform(
            (self.tf_transform.transform.translation.x,
             self.tf_transform.transform.translation.y,
             self.tf_transform.transform.translation.z),
            (self.tf_transform.transform.rotation.x,
             self.tf_transform.transform.rotation.y,
             self.tf_transform.transform.rotation.z,
             self.tf_transform.transform.rotation.w),
            rospy.Time.now(),
            self.tf_transform.child_frame_id,
            self.tf_transform.header.frame_id)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("material_detect_node")
    action = Material_Identify()


    rospy.spin()