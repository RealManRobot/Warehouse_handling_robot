#!/usr/bin/env python3
# -*- coding=UTF-8 -*-

import numpy as np
import math
# ROS库
import rospy
# 导入std_msgs库
from std_msgs.msg import String
# 导入thread模块
import threading

# 四元数计算
import calculate_position as cp

# 相机识别服务端
from cam_msgs.srv import DetectResult, DetectResultRequest
# 控制相机开启关闭
from d435_ros.srv import CameraSetting, CameraSettingRequest
# 机械臂控制、状态、坐标系设置、夹爪、升降、API版本
from dual_arm_msgs.msg import Plan_State, MoveJ, MoveJ_P, MoveL, ChangeWorkFrame_Name, \
    ChangeTool_Name, Gripper_Pick, Lift_Height, Lift_Speed, Gripper_Set

# 头部舵机状态、控制
from servo_ros.msg import ServoAngle, ServoMove

# 底盘状态、导航模式、电源、导航点
from woosh_msgs.msg import RobotStatus, Battery
from woosh_msgs.srv import ExecTask, ExecTaskRequest

from geometry_msgs.msg import Pose


class Robot_Action:

    def __init__(self):
        # 设置末端位姿计算器
        self.calculator = cp.Calculate_Position()

        # 设置机械臂规划状态监听
        self._left_is_plan_succeed = True
        self._right_is_plan_succeed = True

        # 机器人等待事件属性
        self._wait_navigation_event = threading.Event()  # 等待导航事件
        self._wait_manual_event = threading.Event()  # 等待手动模式事件
        self._wait_auto_event = threading.Event()  # 等待自动模式事件
        # **********事件状态控制变量********
        # 机器人模式控制
        self.robot_mode = 0
        # 监听电池电量状态
        self._auto_electricize = 0
        self._manual_electricize = False
        # 监听底盘任务状态
        self._woosh_task_is_succeed = True

        # 参数服务器参数
        self._get_rosparam()

        # 控制器版本获取监听者
        rospy.loginfo(f"The handling robot version is *v1.0.0*.")

        # 相机识别客户端
        self.detect_client = rospy.ServiceProxy("/get_detect_result", DetectResult)
        # 相机开启与关闭客户端
        # self.camera_client = rospy.ServiceProxy("/camera/driver", CameraSetting)

        # 机械臂相关ros通讯配置
        # 机械臂规划状态监听
        rospy.Subscriber("/r_arm/rm_driver/Plan_State", Plan_State, self._plan_right_state, queue_size=1)
        rospy.Subscriber("/l_arm/rm_driver/Plan_State", Plan_State, self._plan_left_state, queue_size=1)
        # 机械臂MoveJ控制
        self.right_movej_pub = rospy.Publisher("/r_arm/rm_driver/MoveJ_Cmd", MoveJ, queue_size=10)
        self.left_movej_pub = rospy.Publisher("/l_arm/rm_driver/MoveJ_Cmd", MoveJ, queue_size=10)
        # 机械臂MoveJ_P控制
        self.right_movejp_pub = rospy.Publisher("/r_arm/rm_driver/MoveJ_P_Cmd", MoveJ_P, queue_size=10)
        self.left_movejp_pub = rospy.Publisher("/l_arm/rm_driver/MoveJ_P_Cmd", MoveJ_P, queue_size=10)
        # 机械臂MoveL控制
        self.right_movel_pub = rospy.Publisher("/r_arm/rm_driver/MoveL_Cmd", MoveL, queue_size=10)
        self.left_movel_pub = rospy.Publisher("/l_arm/rm_driver/MoveL_Cmd", MoveL, queue_size=10)
        # 机械臂工具坐标系设置
        self.set_right_tool_frame = rospy.Publisher("/r_arm/rm_driver/ChangeToolName_Cmd", ChangeTool_Name, queue_size=10)
        self.set_left_tool_frame = rospy.Publisher("/l_arm/rm_driver/ChangeToolName_Cmd", ChangeTool_Name, queue_size=10)
        # 力控夹爪控制
        self.right_gripper_pub = rospy.Publisher("/r_arm/rm_driver/Gripper_Pick_On", Gripper_Pick, queue_size=10)
        self.left_gripper_pub = rospy.Publisher("/l_arm/rm_driver/Gripper_Pick_On", Gripper_Pick, queue_size=10)
        # 开合夹爪控制
        self.right_gripper_pos = rospy.Publisher("/r_arm/rm_driver/Gripper_Set", Gripper_Set, queue_size=10)
        self.left_gripper_pos = rospy.Publisher("/l_arm/rm_driver/Gripper_Set", Gripper_Set, queue_size=10)
        # 升降高度
        self.lift_height_pub = rospy.Publisher("/l_arm/rm_driver/Lift_SetHeight", Lift_Height, queue_size=10)
        self.lift_speed_pub = rospy.Publisher("/l_arm/rm_driver/Lift_SetSpeed", Lift_Speed, queue_size=10)
        # 舵机角度设置、读取
        self.servo_set_angle = rospy.Publisher("/servo_control/move", ServoMove, queue_size=10)
        rospy.Subscriber("/servo_state", ServoAngle, self._get_servo_angle, queue_size=10)
        
        # 底盘状态、电池电量订阅
        rospy.Subscriber("/robot_status", RobotStatus, self._woosh_status, queue_size=10)
        rospy.Subscriber("/battery", Battery, self._woosh_battery, queue_size=10)
        # # 底盘导航模式切换
        # self.navigation_mode_goal = rospy.Publisher("/navigation_mode/goal", NavigationModeGoal, queue_size=10)
        # 底盘导航点位请求
        self.navigation_point = rospy.ServiceProxy("/exec_task", ExecTask)

        # 初始化动作
        self._init_action()
        rospy.sleep(5)
        # 创建任务执行线程
        self.action_thread = threading.Thread(target=self.execute)
        self.action_thread.start()
    
    def __del__(self):
        self.action_thread.join()

    def _init_action(self):
        msg = ChangeTool_Name()
        # 初始化工具坐标系
        msg.toolname = self.left_tool3
        self.set_left_tool_frame.publish(msg)
        msg.toolname = self.right_tool3
        self.set_right_tool_frame.publish(msg)
    
    # 获取ros参数服务器参数
    def _get_rosparam(self):
        # 物料框位置
        self.cargo_boxes = list()
        self.cargo_boxes.append(rospy.get_param("~cargo_box1", "11"))
        self.cargo_boxes.append(rospy.get_param("~cargo_box2", "12"))
        self.cargo_boxes.append(rospy.get_param("~cargo_box3", "21"))
        self.cargo_boxes.append(rospy.get_param("~cargo_box4", "22"))
        # 物料位置
        self.cargoes = list()
        self.cargoes.append(rospy.get_param("~cargo1", "101"))
        self.cargoes.append(rospy.get_param("~cargo2", "102"))
        self.cargoes.append(rospy.get_param("~cargo3", "103"))
        self.cargoes.append(rospy.get_param("~cargo4", "104"))

        # 工具坐标系名称
        self.left_tool1 = rospy.get_param("~left_tool1", "grip_left")
        self.left_tool2 = rospy.get_param("~left_tool2", "nip_left")
        self.left_tool3 = rospy.get_param("~left_tool3", "tip_left")
        self.right_tool1 = rospy.get_param("~right_tool1", "grip_right")
        self.right_tool2 = rospy.get_param("~right_tool2", "nip_right")
        self.right_tool3 = rospy.get_param("~right_tool3", "tip_right")
        
        # 升降高度_auto_electricize
        self.lift_height = {"1": int(rospy.get_param("~floor1", 0.5) * 1000), 
                            "2": int(rospy.get_param("~floor2", 0.5) * 1000), 
                            "3": int(rospy.get_param("~floor3", 0.5) * 1000), 
                            "4": int(rospy.get_param("~floor4", 0.5) * 1000),
                            "5": int(rospy.get_param("~lift_send", 0.25) * 1000),
                            "6": int(rospy.get_param("~lift_table", 0.09) * 1000)}
        
        # 标签名
        self.label_name = {1: rospy.get_param("~label1", "AnNiu"), 
                           2: rospy.get_param("~label2", "XinLing"), 
                           3: rospy.get_param("~label3", "NSK"), 
                           4: rospy.get_param("~label4", "YiHeDa"),
                           5: rospy.get_param("~label_ar", "Aruco")}
        
        # 堵塞机制参数说明
        self.auto_mode_time = rospy.get_param("~auto_mode_time", 300)
        self.manual_mode_time = rospy.get_param("~manual_mode_time", 86400)
        # 自主充电电量范围设置
        self.charge_range = rospy.get_param("~change_range", [30, 90])
        self.charge_pose = rospy.get_param("~charge_pose", "120")

    # 监听底盘电量状态
    def _woosh_battery(self, msg: Battery):
        if msg.batteryPercentage <= self.charge_range[0]:
            self._auto_electricize = 1
        elif msg.batteryPercentage >= self.charge_range[1]:
            self._auto_electricize = 0
        else:
            pass

    # 监听机器人状态
    def _woosh_status(self, msg: RobotStatus):
        # 监听动作是否执行完成
        if msg.task_state == 7:
            self._woosh_task_is_succeed = True
        else:
            self._woosh_task_is_succeed = False

        # 监听机器人模式
        self.robot_mode = msg.robot_mode


    # 监听left机械臂规划状态
    def _plan_left_state(self, msg: Plan_State):
        if msg.state:
            self._left_is_plan_succeed = True
        else:
            self._left_is_plan_succeed = False
    
    # 监听right机械臂规划状态
    def _plan_right_state(self, msg: Plan_State):
        if msg.state:
            self._right_is_plan_succeed = True
        else:
            self._right_is_plan_succeed = False

    # 等待机械臂执行完毕
    def _wait_arm_planned(self, type: int):
        """
        type: 1(left), 2(right), 3(double)
        """
        if type == 1:
            self._left_is_plan_succeed = False
            while not rospy.is_shutdown():
                if self._left_is_plan_succeed:
                    break
                rospy.sleep(0.1)
        elif type == 2:
            self._right_is_plan_succeed = False
            while not rospy.is_shutdown():
                if self._right_is_plan_succeed:
                    break
                rospy.sleep(0.1)
        elif type == 3:
            self._right_is_plan_succeed = False
            self._left_is_plan_succeed = False
            while not rospy.is_shutdown():
                if self._right_is_plan_succeed and self._left_is_plan_succeed:
                    break
                rospy.sleep(0.1)
        else:
            pass

    # 得到舵机角度信息
    def _get_servo_angle(self, msg: ServoAngle):
        data = ServoMove()
        
        if msg.angle_1 > 320 and msg.angle_1 < 360:
            pass
        else:
            data.servo_id = 1
            data.angle = 340
            self.servo_set_angle.publish(data)
        
        if msg.angle_2 > 480 and msg.angle_2 < 520:
            pass
        else:
            data.servo_id = 2
            data.angle = 500
            self.servo_set_angle.publish(data)

    # 双臂夹取物料框
    def _pick_up_the_box(self, detect_result):
        if detect_result.header.frame_id != '':
            # 规划到物体上方
            q, v_l, v_r = self._ar_tf_result(detect_result.pose, [-0.17, -0.0855, 0.097]) # -0.17, -0.0855, 0.09
            self._movejp_plan(q, v_l, 1)
            self._movejp_plan(q, v_r, 2)
            # 等待双臂规划目标物上方完成
            self._wait_arm_planned(3)

            # 规划到物体两侧
            q, v_l, v_r = self._ar_tf_result(detect_result.pose, [-0.17, -0.02, 0.083])
            self._movejp_plan(q, v_l, 1)
            self._movejp_plan(q, v_r, 2)
            # 等待双臂规划目标物上方完成
            self._wait_arm_planned(3)

            # 规划到物体
            q, v_l, v_r = self._ar_tf_result(detect_result.pose, [-0.142, -0.02, 0.083])
            self._movejp_plan(q, v_l, 1)
            self._movejp_plan(q, v_r, 2)
            # 等待双臂规划目标物完成
            self._wait_arm_planned(3)
        else: # TODO
            # 延时0.5s
            rospy.sleep(0.5)
            pass
    
    # 监听机器人的手动状态回调函数
    def _monitor_manual_status(self):
        if self.robot_mode != 1:
            self._wait_manual_event.set()
            rospy.loginfo(f"Switch to the manual mode!")
        else:
            pass
    
    # 监听机器人自动状态回调函数
    def _monitor_auto_status(self):
        if self._auto_electricize == 2:
            if self.robot_mode == 1:    # 自动模式, 进行自主充电
                self._navigation_plan(self.charge_pose, task_type=3)
                self._auto_electricize = 1  # 切换到该充电等级
            else:   # 非自动模式则等待切换成自动模式进行充电
                pass
        elif self._auto_electricize == 1:   # 正在充电
            if self.robot_mode == 2:    # 切换成手动模式时将充电设为零
                self._auto_electricize = 0
            else:
                pass    # 其他则等待充满点自主变成0或者等待手动模式主动设置成零
        elif self._auto_electricize == 0:   # 不进行自主充电
            if self.robot_mode == 1:    # 等待切换成自动模式
                self._wait_auto_event.set()  # 开始执行动作
            else:
                pass
    
    # ------------------------------------总执行程序
    def execute(self):
        rospy.loginfo("Start the whole action ...")
        
        # 机械臂规划至加取姿态
        self._movej_plan([-70, -45, -100, 30, 60, 55], 1)
        self._movej_plan([70, 45, 100, -30, -60, -55], 2)
        self._wait_arm_planned(3)   # 等待两规划停止

        # 规划升降机
        self._set_lift_height(300, 60)
        self._wait_arm_planned(1)

        # 更改工具坐标系
        coor_name = ChangeTool_Name()
        coor_name.toolname = self.left_tool2
        self.set_left_tool_frame.publish(coor_name)
        coor_name.toolname = self.right_tool2
        self.set_right_tool_frame.publish(coor_name)
        rospy.logerr("To detect the box!")
        # 识别AR码
        # 读取头部相机识别结果
        ar_result = self._get_detect_result("camera_middle", "Aruco", 10.0)

        # 抓取物料
        self._pick_up_the_box(ar_result)

        # 升降机上移
        self._set_lift_height(600, 60)

        # 回到机械臂正确收回位置
        self._movej_plan([-81.654, -71.927, -89.154, 8.213, 70.918, -49.307], 1)
        self._movej_plan([81.654, 71.927, 89.154, -8.213, -70.918, 49.307], 2)
        self._wait_arm_planned(3)   # 等待两规划停止

        self._wait_arm_planned(1)


    # 抓取物料
    def _grip_materiel(self, com: str, type: int):
        if com == "close":
            msg = Gripper_Pick()
            msg.speed = 888
            msg.force = 888
            if type == 1:
                self.left_gripper_pub.publish(msg)
            elif type == 2:
                self.right_gripper_pub.publish(msg)
            else:
                pass
        elif com == "open":
            msg = Gripper_Set()
            msg.position = 1000
            if type == 1:
                self.left_gripper_pos.publish(msg)
            elif type == 2:
                self.right_gripper_pos.publish(msg)
            else:
                pass
        else:
            pass
    
    # 得到相机识别结果
    def _get_detect_result(self, link_name: str, label: str, delay: float):
        # 读取头部相机识别结果
        req = DetectResultRequest()
        req.delayed = delay
        req.label = label
        req.link_name = link_name
        result = self.detect_client(req)
        return result
    
    # 处理物料识别结果
    def materiel_detect_result(self, pose: Pose, type: int):
        # 旋转四元数
        q = [pose.orientation.w, 
             pose.orientation.x, 
             pose.orientation.y, 
             pose.orientation.z]
        v = [0, 0, 0]
        if type == 1:
            # 赋值语句
            v[0] = pose.position.x - 0.06106    # -0.06106: base_left->base_middle的位置
            v[1] = pose.position.y
            v[2] = pose.position.z
        elif type == 2:
            # 赋值语句
            v[0] = pose.position.x + 0.06106    # 0.06106: base_right->base_middle的位置
            v[1] = pose.position.y
            v[2] = pose.position.z
        else:
            pass
        
        return q, v


    # 处理AR码识别结果
    def _ar_tf_result(self, pose: Pose, vector: list):
        # 旋转四元数
        q = [pose.orientation.w, 
             pose.orientation.x, 
             pose.orientation.y, 
             pose.orientation.z]

        print("\n", "^" * 20)
        print("orientation: ", q)
        print("position: ", [pose.position.x, pose.position.y, pose.position.z])
        
        vector_l = list()
        vector_l.append(vector[0])
        vector_l.append(vector[1])
        vector_l.append(vector[2])
        vectored = self.calculator.transform_vector_by_quaternion(q, vector_l)

        print("vectored_l: ", vectored)

        # 赋值语句
        v_l = [0, 0, 0]
        v_l[0] = pose.position.x + vectored[0] - 0.06106    # -0.06106: base_left->base_middle的位置
        v_l[1] = pose.position.y + vectored[1]
        v_l[2] = pose.position.z + vectored[2]

        vector_r = list()
        vector_r.append(-vector[0])
        vector_r.append(vector[1])
        vector_r.append(vector[2])
        vectored = self.calculator.transform_vector_by_quaternion(q, vector_r)

        print("vectored_r: ", vectored)

        # 赋值语句
        v_r = [0, 0, 0]
        v_r[0] = pose.position.x + vectored[0] + 0.06106    # 0.06106: base_right->base_middle的位置
        v_r[1] = pose.position.y + vectored[1]
        v_r[2] = pose.position.z + vectored[2]
        print("*" * 20, "\n")

        return q, v_l, v_r


    # 升降装置执行系统
    def _set_lift_height(self, height, speed: 10.0):
        msg = Lift_Height()
        msg.height = height
        msg.speed = speed
        self.lift_height_pub.publish(msg)
        pass

    # ros时间戳回调汉书
    def _ros_timestemp_listen(self):
        
        if self._woosh_task_is_succeed:
            self._wait_navigation_event.set()
            rospy.loginfo("Navigation was successful!")
        else:
            pass
    
    # 规划底盘导航运动
    def _navigation_plan(self, mark_no: str, task_id: int=1, task_exect: int=1, 
                         task_type: int=1, direction: int=0):
        req = ExecTaskRequest()
        req.task_id = task_id
        req.task_exect = task_exect
        req.task_type = task_type
        req.direction = direction
        req.task_type_no = 0
        req.mark_no = mark_no
        res = self.navigation_point(req)

        self._woosh_task_is_succeed = False  # 避免底盘状态改变之后自己来不及置零
        self._wait_navigation_event.clear()
        # 创建时间戳
        rospy.Timer(rospy.Duration(0.5), self._ros_timestemp_listen, oneshot=False)
        if res.success:
            timeout = 120
            if not self._wait_navigation_event.wait(timeout=timeout):
                rospy.logerr(f"Navigation execution timeout (greater than { timeout } seconds!)")
                return False
            else:
                rospy.loginfo(f"Navigation execution was successful!")
            return True
        else:
            rospy.logerr(f"Navigation execution command error!")
            return False

    # movej规划
    def _movej_plan(self, joints: list, type: int, speed: int=5):
        msg = MoveJ()
        
        for joint in joints:
            msg.joint.append(math.radians(joint))

        msg.speed = speed / 100.0
        msg.trajectory_connect = False
        if type == 1:
            self.left_movej_pub.publish(msg)
        elif type == 2:
            self.right_movej_pub.publish(msg)
        else:
            pass
    
    # movejp规划
    def _movejp_plan(self, orientation: list, pose: list, type: int, speed: int=5):
        msg = MoveJ_P()
        msg.Pose.position.x = pose[0]
        msg.Pose.position.y = pose[1]
        msg.Pose.position.z = pose[2]
        msg.Pose.orientation.w = orientation[0]
        msg.Pose.orientation.x = orientation[1]
        msg.Pose.orientation.y = orientation[2]
        msg.Pose.orientation.z = orientation[3]
        
        print(pose, orientation)

        msg.speed = speed / 100.0
        msg.trajectory_connect = False
        if type == 1:
            self.left_movejp_pub.publish(msg)
        elif type == 2:
            self.right_movejp_pub.publish(msg)
        else:
            pass

    # movel规划
    def _movel_plan(self, pose: list, orientation: list, type: int, speed: int=5):
        msg = MoveL()
        msg.Pose.position.x = pose[0]
        msg.Pose.position.y = pose[1]
        msg.Pose.position.z = pose[2]
        msg.Pose.orientation.x = orientation[0]
        msg.Pose.orientation.y = orientation[1]
        msg.Pose.orientation.z = orientation[2]
        msg.Pose.orientation.w = orientation[3]
        
        msg.speed = speed
        msg.trajectory_connect = False
        if type == 1:
            self.left_movel_pub.publish(msg)
        elif type == 2:
            self.right_movel_pub.publish(msg)
        else:
            pass


if __name__=='__main__':
    rospy.init_node("body_handling_action_node")
    robot = Robot_Action()
    rospy.spin()