import cv2
import pyrealsense2 as rs
import numpy as np

import rospy
from cv_bridge import CvBridge

from d435_ros.msg import RealsenseImage
from d435_ros.srv import CameraSetting, CameraSettingRequest, CameraSettingResponse
from d435_ros.srv import PixelToCoordinate, PixelToCoordinateRequest, PixelToCoordinateResponse

# 相机名称字典
CAMERA_NAME = {
    "camera_left": '152122078151',
    "camera_right": '152222072647',
    "camera_middle": '052522070156'
}


class Camera_Info():

    def __init__(self):
        rospy.init_node("D435_show", disable_signals=True)    # 初始化节点

        # 获取相机参数服务器参数
        resolution_width = rospy.get_param("~resolution_width", 640)
        resolution_height = rospy.get_param("~resolution_height", 480)
        frame_rate = rospy.get_param("~frame_rate", 30)

        # 创建发布者对象
        self.object_pub = rospy.Publisher("/camera/image_raw", RealsenseImage, queue_size=1)
        # 相机控制服务器
        self.server_camera = rospy.Service("/camera/driver", CameraSetting, self.set_camera_source)

        # 配置 RealSense
        self.pipeline = rs.pipeline()
        self.rs_config = rs.config()

        self.rs_config.enable_stream(rs.stream.depth, resolution_width, resolution_height, rs.format.z16, frame_rate)
        self.rs_config.enable_stream(rs.stream.color, resolution_width, resolution_height, rs.format.bgr8, frame_rate)
        
        align_to = rs.stream.color  # 与color流对齐
        self.align = rs.align(align_to)

        # 相机控制参数
        self.camera_is_ok = False
        self.frame_id = ""
        self.aligned_depth_frame = None
        self._color_intrinsics = None

    def set_camera_source(self, req: CameraSettingRequest):
        # 初始化相机服务数据
        res = CameraSettingResponse()

        # 如果请求是打开相机
        if req.run:
            if self.frame_id == req.frame_id:
                res.result = True
                res.message = "The device has been opened."
            else:
                # 如果当前有其他相机正在运行，先停止
                if self.frame_id != "":
                    try:
                        self.pipeline.stop()
                    except Exception as err:
                        rospy.logerr(f"Failed to stop pipeline: {err}")
                        res.result = False
                        res.message = f"Failed to stop pipeline: {err}"
                        return res

                try:
                    # 设置打开的相机
                    try:
                        if self.pipeline.is_alive():  # 检查是否正在运行
                            self.pipeline.stop()      # 停止对象
                    except Exception as err:
                        pass

                    self.rs_config.enable_device(CAMERA_NAME[req.frame_id])
                    # 启动相机流
                    profile = self.pipeline.start(self.rs_config)
                    self._color_intrinsics = (
                        profile.get_stream(rs.stream.color)
                        .as_video_stream_profile()
                        .get_intrinsics()
                    )
                    self.camera_is_ok = True
                    res.result = True
                    res.message = "The device was opened successfully."
                    self.frame_id = req.frame_id
                except KeyError as e:
                    res.result = False
                    res.message = f"Invalid frame_id: {req.frame_id}"
                    rospy.logerr(f"Invalid frame_id: {e}")
                except Exception as e:
                    self.camera_is_ok = False
                    res.result = False
                    res.message = f"Unexpected error: {e}"
                    rospy.logerr(f"Unexpected error: {e}")

        # 如果请求是关闭相机
        elif req.frame_id == self.frame_id:
            try:
                self.pipeline.stop()
                res.result = True
                res.message = "The device was closed successfully."
                self.frame_id = ""
                self.camera_is_ok = False
            except Exception as err:
                res.result = False
                res.message = f"Failed to stop pipeline: {err}"
                rospy.logerr(f"Failed to stop pipeline: {err}")

        # 如果请求的相机未打开
        else:
            res.result = False
            res.message = "The device is not turned on!"

        # 输出结果
        rospy.loginfo(res.message)
        return res

    def get_aligned_images(self):
        if self.camera_is_ok:
            frames = self.pipeline.wait_for_frames()  # 等待获取图像帧
            aligned_frames = self.align.process(frames)  # 获取对齐帧
            self.aligned_depth_frame = aligned_frames.get_depth_frame()  # 获取对齐帧中的depth帧
            color_frame = aligned_frames.get_color_frame()  # 获取对齐帧中的color帧

            # 获取相机内参
            color_profile = frames.get_color_frame().get_profile()
            intrinsics = color_profile.as_video_stream_profile().get_intrinsics()

            depth_image = np.asanyarray(self.aligned_depth_frame.get_data())  # 深度图（默认16位）
            color_image = np.asanyarray(color_frame.get_data())  # RGB图

            # 返回相机内参、深度参数、彩色图、深度图、齐帧中的depth帧
            return color_image, depth_image, intrinsics
        else:
            return np.array([]), np.array([]), None
    
    def get_internal_reference(self, intrinsics):
        # 提取相机内参
        camera_matrix = np.array([[intrinsics.fx, 0, intrinsics.ppx],
                                [0, intrinsics.fy, intrinsics.ppy],
                                [0, 0, 1]], dtype=np.float32)

        dist_coeffs = np.array(intrinsics.coeffs, dtype=np.float32)
        return camera_matrix, dist_coeffs

    def camera_publish(self):
        bridge = CvBridge()
        seq = 0
        rate = rospy.Rate(5)
        try:
            while not rospy.is_shutdown():
                # 等待获取一对连续的帧：深度和颜色
                try:
                    color_image, depth_image, intrinsics = self.get_aligned_images()
                except Exception as err:
                    rate.sleep()
                    continue
        
                if intrinsics is None or not depth_image.any() or not color_image.any():
                    rospy.sleep(0.1)
                    seq = 0
                    continue

                realsense_images = RealsenseImage()

                # 将相机内参转换成ROS相机内参消息
                camera_matrix, dist_coeffs = self.get_internal_reference(intrinsics)
                realsense_images.camera_info.header.frame_id = self.frame_id
                realsense_images.camera_info.header.stamp = rospy.Time.now()
                realsense_images.camera_info.header.seq = seq
                realsense_images.camera_info.height = intrinsics.height
                realsense_images.camera_info.width = intrinsics.width
                realsense_images.camera_info.K = camera_matrix.flatten().tolist()
                realsense_images.camera_info.D = dist_coeffs.tolist()
                
                # 将OpenCV图像转换为ROS图像消息
                color_msg = bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
                color_msg.header.frame_id = self.frame_id
                color_msg.header.seq = seq
                color_msg.header.stamp = rospy.Time.now()

                depth_msg = bridge.cv2_to_imgmsg(depth_image, encoding="mono16")
                depth_msg.header.frame_id = self.frame_id
                depth_msg.header.seq = seq
                depth_msg.header.stamp = rospy.Time.now()

                realsense_images.rgb = color_msg
                realsense_images.deep = depth_msg
                #发布图像
                self.object_pub.publish(realsense_images)
                seq += 1

                # 释放资源
                del color_image, depth_image, intrinsics, color_msg, depth_msg, realsense_images

                # 时间等待函数
                rate.sleep()
        except Exception as err:
            rospy.logerr(str(err))
            # 停止流
            try:
                self.pipeline.stop()
            except Exception as err:
                pass


if __name__ == "__main__":
    # 实例化相机对象
    camera_info = Camera_Info()
    # 开始发布参数
    camera_info.camera_publish()
