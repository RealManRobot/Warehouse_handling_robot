#!/usr/bin/env python3
# -*- coding=UTF-8 -*-

import numpy as np
import time
from typing import List
import math
# ROS库
import rospy
import tf
# 导入std_msgs库
from std_msgs.msg import String
# 导入thread模块
import threading
import random

# 四元数计算
import calculate_position as cp

# 相机识别服务端
# from cam_msgs.srv import DetectResult, DetectResultRequest
from cam_msgs.srv import HandResult, HandResultRequest
from cam_msgs.srv import HeadResult, HeadResultRequest

# 控制相机开启关闭
from d435_ros.srv import CameraSetting, CameraSettingRequest
# 机械臂控制、状态、坐标系设置、夹爪、升降、API版本
from dual_arm_msgs.msg import Plan_State, MoveJ, MoveJ_P, MoveL, \
    ChangeTool_Name, Gripper_Pick, Lift_Height, Lift_Speed, Gripper_Set

# 头部舵机状态、控制
from servo_ros.msg import ServoAngle, ServoMove

# 底盘状态、导航模式、电源、导航点
from woosh_msgs.msg import RobotStatus, Battery
from woosh_msgs.srv import ExecTask, ExecTaskRequest
from woosh_msgs.msg import StepControlAction, StepControlGoal, StepControl
import actionlib

from geometry_msgs.msg import Pose


# 打乱物料箱抓取位置
def shuffle_list(col: int, row: int):
    pose_list = list()

    for col_index in range(col):
        for row_index in range(row):
            pose_list.append(str(col_index + 1) + str(row_index + 1))

    # 随机打乱行顺序
    random.shuffle(pose_list)
    print("\n", pose_list)
    
    return pose_list


class Robot_Action:

    def __init__(self):
        # 设置末端位姿计算器
        self.calculator = cp.Calculate_Position()

        # 设置机械臂规划状态监听
        self._left_is_plan_succeed = True
        self._right_is_plan_succeed = True
        
        # 设置物料抓取动作状态
        self.actions_thread_event = { 1: False, 2: False }

        # 机器人等待事件属性
        self._wait_navigation_event = threading.Event()  # 等待导航事件
        self._wait_arm_event = {1: threading.Event(), 2: threading.Event(), 3: threading.Event()}  # 等待机械臂事件
        self._wait_manual_event = threading.Event()  # 等待手动模式事件
        self._wait_auto_event = threading.Event()  # 等待自动模式事件
        self._wait_arm_actions_event = threading.Event()  # 等待线程动作完成

        # 设置线程锁
        # 三个相机识别的线程锁
        self.identify_lock = threading.Lock()
        # 放置物品防碰撞的线程锁
        self.placement_lock = threading.Lock()
        # 双臂等待线程锁
        self.arms_wait_lock = threading.Lock()
        
        # **********事件状态控制变量********
        # 机器人模式控制
        self.robot_mode = 0
        self.robot_state = 0
        self.work_mode = 0
        # 监听电池电量状态
        self._auto_electricize = 0
        self._manual_electricize = False
        # 监听底盘任务状态
        self._woosh_task_is_succeed = True
        # 记录导航到达的点位名称
        self._navigation_point = ""
        # 获取舵机误差
        self._servo_error = [3, 3]
        # 物料框位置
        self.cargo_boxes = list()
        # 物料位置
        self.cargoes = list()

        # 重新定义rospy.loginfo()与rospy.logerr()
        self._talk_flag = False
        self.loginfo = rospy.loginfo
        self.logerr = rospy.logerr
        self.logwarn = rospy.logwarn
        # 设置重构
        rospy.loginfo = self.robot_loginfo
        rospy.logerr = self.robot_logerr
        rospy.logwarn = self.robot_logwarn
        # 控制数据变量初始化
        self.ctrl_dict = dict()

        # 识别状态是否被占用
        self.identify_is_free = True
        # 判断放置位置是否被占用
        self.placement_is_free = True

        # 控制器版本获取监听者
        rospy.loginfo(f"The handling robot version is *v1.0.0*.")

        self.voice_pub = rospy.Publisher("/voice_synthesis", String, queue_size=10)
        # 相机识别客户端
        self.hand_detect_client = rospy.ServiceProxy("/hand_detect_result", HandResult)
        self.head_detect_client = rospy.ServiceProxy("/head_detect_result", HeadResult)
        # 相机开启与关闭客户端
        self.camera_client = rospy.ServiceProxy("/camera/driver", CameraSetting)

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
        self.servo_get_angle = rospy.Subscriber("/servo_state", ServoAngle, self._get_servo_angle, queue_size=10)
        
        # 参数服务器参数
        self._get_rosparam()

        # 底盘状态、电池电量订阅
        rospy.Subscriber("/robot_status", RobotStatus, self._woosh_status, queue_size=10)
        rospy.Subscriber("/battery", Battery, self._woosh_battery, queue_size=10)
        
        # 底盘导航点位请求
        self.navigation_point = rospy.ServiceProxy("/exec_task", ExecTask)

        # 创建Action客户端
        # 创建一个SimpleActionClient，连接到/cmd_vel_control action server
        self.base_step_client = actionlib.SimpleActionClient('/cmd_vel_control', StepControlAction)

        # 初始化动作
        self._init_action()
        
        # 播报程序启动成功指令
        rospy.logwarn("欢迎来到睿尔曼展位，程序成功启动，请技术人员提前做好准备工作！")

        rospy.sleep(10)
        # 创建任务执行线程
        self.action_thread = threading.Thread(target=self.execute)
    
    # 析构函数
    def __del__(self):
        self.action_thread.join()

    # 初始化action配置
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
        cargo_box_col = rospy.get_param("~cargo_box_col", 2)
        cargo_box_row = rospy.get_param("~cargo_box_row", 2)
        # 得到打乱之后的物料框位置
        self.cargo_boxes = shuffle_list(cargo_box_col, cargo_box_row)
        self.cargo_box_num = rospy.get_param("~cargo_box_num", 1)
        self.cargoes_name = rospy.get_param("~cargoes_name", ["JiaoHuanJi", "NSK", "AnNiu", "YiHeDa"])

        for index in range(len(self.cargoes_name)):
            # 不全成1xx类型的名称
            self.cargoes.append("1" + f"{index + 1}".rjust(2, '0')[:3])

        rospy.loginfo(f"Nav pose: { self.cargoes }!")

        # 工具坐标系名称
        self.left_tool1 = rospy.get_param("~left_tool1", "grip_left")
        self.left_tool2 = rospy.get_param("~left_tool2", "nip_left")
        self.left_tool3 = rospy.get_param("~left_tool3", "tip_left")
        self.right_tool1 = rospy.get_param("~right_tool1", "grip_right")
        self.right_tool2 = rospy.get_param("~right_tool2", "nip_right")
        self.right_tool3 = rospy.get_param("~right_tool3", "tip_right")
        
        # 升降高度_auto_electricize
        self.lift_height = {"1": int(rospy.get_param("~floor1", 0.05) * 1000), 
                            "2": int(rospy.get_param("~floor2", 0.45) * 1000), 
                            "3": int(rospy.get_param("~floor3", 0.85) * 1000), 
                            "4": int(rospy.get_param("~floor4", 0.999) * 1000),
                            "5": int(rospy.get_param("~lift_send", 0.6) * 1000),
                            "6": int(rospy.get_param("~lift_table", 0.3) * 1000)}
        
        # 升降控制速度
        self.lift_speed = rospy.get_param("~lift_speed", 50)
        # 机械臂控制速度
        self.arms_speed = rospy.get_param("~arms_speed", 30)
        
        # 是否进行语音播报
        self._talk_flag = rospy.get_param("~talk_flag", True)
        
        # 堵塞机制参数说明
        self.auto_mode_time = rospy.get_param("~auto_mode_time", 86400)
        self.manual_mode_time = rospy.get_param("~manual_mode_time", 86400)
        # 自主充电电量范围设置 TODO 变量名出错
        self.charge_range = rospy.get_param("~charge_range", [30, 90])
        self.charge_pose = rospy.get_param("~charge_pose", "120")
        # 舵机角度设置
        self.servo_motor = rospy.get_param("~servo_motor", [380, 500])
        self.servo_tolerant = rospy.get_param("~servo_tolerant", [3, 3])
        # self.servo_motor = [int(item) for item in rospy.get_param("~servo_motor", [350, 500])]

        # 关节预定义设置
        self.arm_joints = {
            1: rospy.get_param("~left_arm_joints"),
            2: rospy.get_param("~right_arm_joints")
        }
        self.arm_poses = {
            1: rospy.get_param("~left_arm_poses"),
            2: rospy.get_param("~right_arm_poses")
        }

        # 将用到的变量写进字典
        self.ctrl_dict = {
            1: {
                "tools": [self.left_tool1, self.left_tool2, self.left_tool3],
                "tools_pub": self.set_left_tool_frame,
                "joints": [
                    self.arm_joints[1][4],    # 识别姿态
                    self.arm_joints[1][5],    # 过渡姿态
                    self.arm_joints[1][6],    # 放置物料姿态
                ]
            },
            2: {
                "tools": [self.right_tool1, self.right_tool2, self.right_tool3],
                "tools_pub": self.set_right_tool_frame,
                "joints": [
                    self.arm_joints[2][4],    # 识别姿态
                    self.arm_joints[2][5],    # 过渡姿态
                    self.arm_joints[2][6],    # 放置物料姿态
                ]
            }
        }

    # 语音播报模块
    def _robot_talk_about(self, infor: str):
        msg = String()
        msg.data = infor
        self.voice_pub.publish(msg)

    # 错误重载
    def robot_logerr(self, msg):
        if self._talk_flag:
            self._robot_talk_about("出错！" + msg)
        self.logerr(msg)
        pass

    # 警告重载
    def robot_logwarn(self, msg):
        if self._talk_flag:
            self._robot_talk_about(msg)
        self.logwarn(msg)

    # 正常输出重载
    def robot_loginfo(self, msg):
        # if self._talk_flag:
        #     self._robot_talk_about(msg)
        self.loginfo(msg)

    # 监听底盘电量状态
    def _woosh_battery(self, msg: Battery):
        if msg.batteryPercentage <= self.charge_range[0] and \
            not self._auto_electricize and \
            self.robot_state == 2:  # 空闲模式
            # 设置自主充电
            rospy.logwarn(f"机器人电量小于百分之{ self.charge_range[0] }，即将执行自主充电，请将机器人手动移动至合适位置。")
            self._auto_electricize = 1  # 一级充电模式，开始充电
        elif msg.batteryPercentage >= self.charge_range[1]:
            self._auto_electricize = 0  # 停止充电
        else:
            pass
        # rospy.loginfo(f"Charge range: { self.charge_range }.")

        # rospy.loginfo(f"Battery value: { msg.batteryPercentage }, electricize value: { self._auto_electricize }.")

    # 监听机器人状态
    def _woosh_status(self, msg: RobotStatus):
        # 监听动作是否执行完成
        if msg.task_state == 7:
            self._woosh_task_is_succeed = True
        else:
            pass

        # 监听机器人模式
        self.robot_mode = msg.robot_mode
        self.robot_state = msg.robot_state
        self.work_mode = msg.work_mode
        rospy.loginfo(f"{ '*' * 20 }")
        rospy.loginfo(f"[Task  state]:<{msg.task_state}>")
        rospy.loginfo(f"[Robot  mode]:<{msg.robot_mode}>")
        rospy.loginfo(f"[Robot state]:<{msg.robot_state}>")
        rospy.loginfo(f"[Work   mode]:<{msg.work_mode}>")
        rospy.loginfo(f"{ '*' * 20 }")

    # 监听left机械臂规划状态
    def _plan_left_state(self, msg: Plan_State):
        self._left_is_plan_succeed = msg.state
    
    # 监听right机械臂规划状态
    def _plan_right_state(self, msg: Plan_State):
        self._right_is_plan_succeed = msg.state

    # 等待机械臂执行完毕
    def _wait_arm_planned(self, type: int, timeout=20):  # TODO 规划成功判定到机械臂规划之前
        """
        type: 1(left), 2(right), 3(double)
        """
        # self._left_is_plan_succeed = False
        # self._right_is_plan_succeed = False
        # 清除事件应用
        self._wait_arm_event[type].clear()
        
        # 创建时间戳
        timer = rospy.Timer(rospy.Duration(0.5), 
                            callback=lambda event: self._arm_timestemp_listen(
                                arm_type=type
                            ), oneshot=False)
        # 等待机械臂完成规划
        if not self._wait_arm_event[type].wait(timeout=timeout):
            rospy.logerr(f"机械臂未在{ timeout }秒内完成规划，请将物料放置合适位置")
            timer.shutdown()
            return False
        else:
            rospy.loginfo(f"Arm execution was successful!")
            timer.shutdown()
            return True

    # 得到舵机角度信息
    def _get_servo_angle(self, msg: ServoAngle):
        data = ServoMove()
        time.sleep(0.5)
        if msg.angle_1 > self.servo_motor[0] - 3 and msg.angle_1 < self.servo_motor[0] + 3:
            pass
        else:
            data.servo_id = 1
            data.angle = self.servo_motor[0]
            self.servo_set_angle.publish(data)
            # msg.angle_1 = self.servo_motor[0]
        
        if msg.angle_2 > self.servo_motor[1] - 3 and msg.angle_2 < self.servo_motor[1] + 3:
            pass
        else:
            data.servo_id = 2
            data.angle = self.servo_motor[1]
            self.servo_set_angle.publish(data)
            # msg.angle_2 = self.servo_motor[1]
        time.sleep(0.5)
        rospy.loginfo(f"The head angle of the robot is: [{ msg.angle_1 }, { msg.angle_2 }]")
        self._servo_error[0] = abs(msg.angle_1 - self.servo_motor[0])
        self._servo_error[1] = abs(msg.angle_2 - self.servo_motor[1])

        # 单次发布数据，之后杀死话题
        # self.servo_set_angle.unregister()
        self.servo_get_angle.unregister()

    # 双臂夹取物料框
    def _pick_up_the_box(self, detect_result):
        if detect_result.header.frame_id != '':
            # 规划到物体上方
            pose_vector = self._ar_tf_result(detect_result.pose, [0.17, -0.0855, 0.083]) # -0.17, -0.0855, 0.09
            self._movejp_plan(pose_vector[0]["orientation"], pose_vector[0]["position"], 1, speed=self.arms_speed)
            self._movejp_plan(pose_vector[1]["orientation"], pose_vector[1]["position"], 2, speed=self.arms_speed)
            # 等待双臂规划目标物上方完成
            self._wait_arm_planned(3)

            # 规划到物体两侧
            pose_vector = self._ar_tf_result(detect_result.pose, [0.17, -0.02, 0.083])
            self._movejp_plan(pose_vector[0]["orientation"], pose_vector[0]["position"], 1, speed=self.arms_speed)
            self._movejp_plan(pose_vector[1]["orientation"], pose_vector[1]["position"], 2, speed=self.arms_speed)
            # 等待双臂规划目标物上方完成
            self._wait_arm_planned(3)

            # 规划到物体
            pose_vector = self._ar_tf_result(detect_result.pose, [0.14, -0.02, 0.083])
            self._movejp_plan(pose_vector[0]["orientation"], pose_vector[0]["position"], 1, speed=self.arms_speed)
            self._movejp_plan(pose_vector[1]["orientation"], pose_vector[1]["position"], 2, speed=self.arms_speed)
            # 等待双臂规划目标物完成
            self._wait_arm_planned(3)
        else:
            # 延时0.5s
            rospy.sleep(0.5)
            pass
    
    # 监听机器人的手动状态回调函数
    def _monitor_manual_status(self, event):
        if self.robot_mode != 1:
            self._wait_manual_event.set()
            rospy.logwarn("机器人已切换至手动模式！")
        else:
            pass
    
    # 监听机器人自动状态回调函数
    def _monitor_auto_status(self, event):
        if self._auto_electricize == 2:
            if self.robot_mode == 1:    # 等待自动模式, 进行自主充电
                self._navigation_wait(self._navigation_plan(self.charge_pose, task_type=3))
                rospy.logwarn("机器人开始执行自主充电！")
                time.sleep(5)
                self._auto_electricize = 1  # 切换到该充电等级
            else:   # 非自动模式则等待切换成自动模式进行充电
                pass
        elif self._auto_electricize == 1:   # 正在充电
            if self.robot_mode == 2:    # 等待切换成手动模式时将充电设为零
                self._auto_electricize = 0
            else:
                pass    # 其他则等待充满点自主变成0或者等待手动模式主动设置成零
        elif self._auto_electricize == 0:   # 不进行自主充电
            if self.robot_mode == 1:    # 等待切换成自动模式
                self._wait_auto_event.set()  # 开始执行动作
            else:
                pass

        # rospy.loginfo(f"Auto electricize: { self._auto_electricize }")

    # 等待线程执行完毕完成
    def _arms_actions_status(self, event):
        # 线程动作状态
        if self.actions_thread_event[1] and self.actions_thread_event[2]:
            self._wait_arm_actions_event.set()
            # 还原抓取状态
            self.actions_thread_event[1] = False
            self.actions_thread_event[2] = False
        else:
            pass
    
    def _wait_actions_finish(self, timeout: int=3600):

        # 清除动作线程
        self._wait_arm_actions_event.clear()

        # 创建时间戳
        timer = rospy.Timer(rospy.Duration(0.5), self._arms_actions_status, False)
        
        # 等待数据
        if not self._wait_arm_actions_event.wait(timeout=timeout):
            rospy.logerr("动作通讯未按时完成！")
            timer.shutdown()
            return False
        else:
            rospy.loginfo(f"Arm execution was successful!")
            timer.shutdown()
            return True

    # ------------------------------------ 总执行程序
    def execute(self):
        
        # 判断底盘是否可以正常使用
        # 初始化判断次数
        test_times = 0
        while not rospy.is_shutdown():
            
            # 判断机器人工作模式
            if self.work_mode != 2:
                 # 判断, 三次
                if test_times >= 3:
                    time.sleep(1)
                else:
                    rospy.logwarn("请将机器人设置成任务模式！")
                    time.sleep(6)
            elif self.robot_state != 2 and self.robot_state != 8:
                # 判断, 三次
                if test_times >= 3:
                    time.sleep(1)
                else:
                    if self.robot_state == 0:
                        rospy.logwarn("机器人状态未被定义！")
                    elif self.robot_state == 1:
                        rospy.logwarn("机器人未被初始化！")
                    elif self.robot_state == 3:
                        rospy.logwarn("机器人正在泊车！")
                    elif self.robot_state == 4:
                        rospy.logwarn("机器人正在执行任务中，请关闭当前任务！")
                    else:
                        rospy.logwarn("机器人状态异常！")
                    time.sleep(6)
            else:
                break
            test_times += 1

            # 延时
            time.sleep(1)

        # 判断舵机误差
        if self._servo_error[0] >= self.servo_tolerant[0] or self._servo_error[1] >= self.servo_tolerant[1]:
            rospy.logwarn(f"舵机角度需要重新设置，请确保一号舵机值为{ self.servo_motor[0] }, 二号舵机值为{ self.servo_motor[1] }!")
            return False

        rospy.logwarn("程序初始化完毕!")

        # 循环执行动作，直到节点关闭
        while not rospy.is_shutdown():
            
            # ************堵塞机器人************
            # 刷新事件
            self._wait_manual_event.clear()
            
            # 创建时间戳，判断机器人是否处于手动模式状态
            timer = rospy.Timer(rospy.Duration(0.5), self._monitor_manual_status, oneshot=False)
            if not self._wait_manual_event.wait(timeout=self.manual_mode_time):   # 等5分钟
                rospy.logerr(f"{ self.manual_mode_time / 60.0 }分钟内没有切换至手动模式去调整机器人！")
                return False
            else:
                rospy.logwarn("机器人已切换至手动模式，请及时还原案例演示场景！")
            timer.shutdown()

            # 游离升降机位置
            self._set_lift_height(self.lift_height["5"], self.lift_speed)
            self._wait_arm_planned(1)

            # 缩紧机械臂, 作为标志代表接下来需要让机器人先停止，让人工进行操作之后再做搬运动作
            self._double_movej_s(0, speed=self.arms_speed)
            # 双臂抓取
            self._grip_materiel("open", 1)
            self._grip_materiel("open", 2)
            rospy.sleep(3)

            # 判断是否进行自主充电
            if self._auto_electricize:
                # 机器人充电升级，自主充电
                self._auto_electricize = 2

                # 创建事件戳，判断机器人是否处于自动模式状态
                self._wait_auto_event.clear()
                timer = rospy.Timer(rospy.Duration(0.5), self._monitor_auto_status, oneshot=False)
                if not self._wait_auto_event.wait(timeout=self.auto_mode_time):   # 等24小时
                    rospy.logerr(f"{ self.auto_mode_time / 60.0 }分钟内没有充满电，请检查充电装置！")
                    return False
                else:
                    rospy.logwarn("自主充电成功！")
                    time.sleep(5)
                timer.shutdown()
            else:
                # 创建事件戳，判断机器人是否处于自动模式状态
                self._wait_auto_event.clear()
                timer = rospy.Timer(rospy.Duration(0.5), self._monitor_auto_status, oneshot=False)
                if not self._wait_auto_event.wait(timeout=self.auto_mode_time):   # 等24小时
                    rospy.logerr(f"{ self.auto_mode_time / 60.0 }分钟内没有切换至自动模式！")
                    return False
                else:
                    rospy.logwarn("成功切换至自动模式！")
                    time.sleep(5)
                timer.shutdown()

            # 循环遍历n个整体任务
            for index in range(self.cargo_box_num):
                if self._navigation_point != self.cargo_boxes[index][0]:
                    # 到达过渡点位
                    self._navigation_wait(self._navigation_plan(self.cargo_boxes[index][0] + "0"))
                    # print(3)
                else:
                    pass
                
                # 将升降机升降到物料框固定位置上方
                self._set_lift_height(self.lift_height[self.cargo_boxes[index][1]], self.lift_speed)
                self._wait_arm_planned(1)

                # 将底盘规划到待抓取点位
                self._navigation_wait(self._navigation_plan(self.cargo_boxes[index][0]))
                self._navigation_point = self.cargo_boxes[index][0]

                # 将双臂伸进
                self._double_movel_s(0, speed=self.arms_speed)
                # print(4)

                # 将升降机升降到物料框固定位置
                self._set_lift_height(self.lift_height[self.cargo_boxes[index][1]] - 50, self.lift_speed)
                self._wait_arm_planned(1)

                # 更改工具坐标系
                coor_name = ChangeTool_Name()
                coor_name.toolname = self.left_tool2
                for _ in range(3):
                    self.set_left_tool_frame.publish(coor_name)
                    rospy.sleep(0.1)
                coor_name.toolname = self.right_tool2
                for _ in range(3):
                    self.set_right_tool_frame.publish(coor_name)
                    rospy.sleep(0.1)
                # print(5)

                frame_id = "camera_middle"
                # 识别AR码
                if self._set_camera_status(True, frame_id):
                    # 读取头部相机识别结果
                    ar_result = self._get_aruco_detect_result(num=582, link_name=frame_id, delay=10.0)

                    # 夹取物料框
                    self._pick_up_the_box(ar_result)
                    
                    if self._set_camera_status(False, frame_id):
                        pass
                    else:
                        rospy.logerr("头部相机关闭失败，请检查装置！")
                        return False
                else:
                    rospy.logerr("头部相机打开失败，请检察装置！")
                    return False

                # 升降机上移
                self._set_lift_height(self.lift_height[self.cargo_boxes[index][1]] + 30, self.lift_speed)
                self._wait_arm_planned(1)

                # 缩机械臂, 到达目标位
                self._double_movej_s(1, speed=self.arms_speed)
                
                rospy.logwarn(f"机器人将装填第{ self.cargo_boxes[index][0] }列，第{ self.cargo_boxes[index][1] }层的物料框！")

                # 控制底盘后退
                if not self._base_step_plan(-0.3):
                    rospy.logerr("机器人步进服务回调失败！")
                # 行走至物料目标位
                carry = self._navigation_plan(self.cargoes[index])
                # 延时6秒执行, 防止碰到货架
                rospy.sleep(5)
                
                # 升降机移动至运送位
                self._set_lift_height(self.lift_height["5"], self.lift_speed)
                self._wait_arm_planned(1)
                self._navigation_wait(carry=carry)

                # 双臂放置盒子
                self._double_movej_s(2, speed=self.arms_speed)

                # 升降机至桌子上
                self._set_lift_height(self.lift_height["6"] - 35, self.lift_speed)
                self._wait_arm_planned(1)
            
                # 双臂拿开
                self._double_movej_s(3, speed=self.arms_speed)

                # 升降机至识别位置
                self._set_lift_height(self.lift_height["6"] + 50, self.lift_speed)
                self._wait_arm_planned(1)

                # 双臂至识别目标物位姿
                self._double_movej_s(4, speed=self.arms_speed)

                # 升降机至抓取位置
                self._set_lift_height(self.lift_height["6"], self.lift_speed)
                self._wait_arm_planned(1)
                # 延迟数据
                rospy.sleep(1)

                # 线程抓取物料到物料箱子
                self.left_thread = threading.Thread(target=self._arm_actions_of_thread, args=("camera_left", 1, index, self.arms_speed))
                self.right_thread = threading.Thread(target=self._arm_actions_of_thread, args=("camera_right", 2, index, self.arms_speed))
                
                # 手臂线程开始
                self.left_thread.start()
                self.right_thread.start()

                # 等待抓取到物料框完成
                self._wait_actions_finish(3600)

                # 拿物料箱
                # 更改工具坐标系
                coor_name = ChangeTool_Name()
                coor_name.toolname = self.left_tool2
                for _ in range(3):
                    self.set_left_tool_frame.publish(coor_name)
                    rospy.sleep(0.1)
                
                coor_name.toolname = self.right_tool2
                for _ in range(3):
                    self.set_right_tool_frame.publish(coor_name)
                    rospy.sleep(0.1)

                # 升降机至桌子上方
                self._set_lift_height(self.lift_height["6"] + 50, self.lift_speed)
                self._wait_arm_planned(1)

                # 将机械臂伸进
                self._double_movej_s(3, speed=self.arms_speed)

                # 升降机至桌子上
                self._set_lift_height(self.lift_height["6"], self.lift_speed)
                self._wait_arm_planned(1)

                frame_id = "camera_middle"
                # 识别AR码
                if self._set_camera_status(True, frame_id):
                    # 读取头部相机识别结果
                    ar_result = self._get_aruco_detect_result(num=582, link_name=frame_id, delay=10.0)

                    # 夹取物料框
                    self._pick_up_the_box(ar_result)
                    
                    if self._set_camera_status(False, frame_id):
                        pass
                    else:
                        rospy.logerr("头部相机关闭失败，请检查装置！")
                        return False
                else:
                    rospy.logerr("头部相机打开失败，请检察装置！")
                    return False

                # 放回货架
                # 升降机上移至运送位
                self._set_lift_height(self.lift_height["5"], self.lift_speed)
                self._wait_arm_planned(1)

                # 缩机械臂
                self._double_movej_s(7, speed=self.arms_speed)

                # 控制底盘后退
                if not self._base_step_plan(-0.15):
                    rospy.logerr("机器人步进服务回调失败！")

                # 等待升降机至拿货位
                self._navigation_wait(self._navigation_plan(self.cargo_boxes[index][0] + "0"))

                # 升降机至货架位
                self._set_lift_height(self.lift_height[self.cargo_boxes[index][1]], self.lift_speed)
                self._wait_arm_planned(1)

                # 行走至货架目标位
                self._navigation_wait(self._navigation_plan(self.cargo_boxes[index][0]))

                # 将装好的框放置在货架上去伸机械臂 
                self._double_movej_s(8, speed=self.arms_speed)

                # 升降机至放置位
                self._set_lift_height(self.lift_height[self.cargo_boxes[index][1]] - 95, self.lift_speed)
                self._wait_arm_planned(1)
                # 双臂拿开
                self._double_movej_s(3, speed=self.arms_speed)
                # 升降机至货架位
                self._set_lift_height(self.lift_height[self.cargo_boxes[index][1]], self.lift_speed)
                self._wait_arm_planned(1)
                # 缩紧机械臂
                self._double_movel_s(1, speed=self.arms_speed)

                # 控制底盘后退
                if not self._base_step_plan(-0.3):
                    rospy.logerr("机器人步进服务回调失败！")
                
                rospy.logwarn(f"第{ index + 1 }次任务执行成功！")

            # 重新一轮demo演示，将已到达的位置初始化
            self._navigation_point = ""
    
    # 机械臂movej双臂控制
    def _double_movej_s(self, index, speed: int=10):
        rospy.loginfo(f"{self.arm_joints[1][index]}")
        self._movej_plan(self.arm_joints[1][index], 1, speed)
        self._movej_plan(self.arm_joints[2][index], 2, speed)
        self._wait_arm_planned(3)

    # 机械臂movel双臂规划
    def _double_movel_s(self, index, speed: int=20):

        self._movel_plan(self.arm_poses[1][index][0], self.arm_poses[1][index][1], 1, speed)
        self._movel_plan(self.arm_poses[2][index][0], self.arm_poses[2][index][1], 2, speed)
        self._wait_arm_planned(3)

    # 双臂抓取物料时的动作线程
    def _arm_actions_of_thread(self, frame_id: str, ctrl_type: int, cargoe_index: int, arm_speed: int=10):
        if ctrl_type == 1:
            arm_name = "左臂"
        else:
            arm_name = "右臂"
        
        # 初始化次数
        index = 0
        # 添加循环执行
        while index < 4:

            # 切换左臂工具坐标系至tip，进行识别
            coor_name = ChangeTool_Name()
            coor_name.toolname = self.ctrl_dict[ctrl_type]["tools"][2]
            for _ in range(3):
                self.ctrl_dict[ctrl_type]["tools_pub"].publish(coor_name)
                rospy.sleep(0.1)
            # 初始化默认值
            detect_result = None
            # 打开相机(加上线程锁)
            with self.identify_lock: 
                if self._set_camera_status(True, frame_id):
                    # 执行视觉识别得到结果
                    # 得到识别结果
                    detect_result = self._get_material_detect_result(self.cargoes_name[cargoe_index], frame_id, 20.0)
                    rospy.loginfo(f"Detect result =>: {self.cargoes_name[cargoe_index]}")

                    if self._set_camera_status(False, frame_id):
                        pass
                    else:
                        rospy.logerr(f"{arm_name}相机关闭失败，请检查装置！")
                        exit(0)
                else:
                    rospy.logerr(f"{arm_name}相机打开失败，请检察装置！")
                    exit(0)

            # 分析物料识别动作
            q_, v_ = self._analysis_materiel_detect_result(detect_result.pose)
            if all(v__ == 0.0 for v__ in v_):
                rospy.logwarn(f"{arm_name}识别数据为零！")
                continue

            # 切换工具坐标系至grip_left, 进行抓取
            coor_name = ChangeTool_Name()
            coor_name.toolname = self.ctrl_dict[ctrl_type]["tools"][0]
            for _ in range(3):
                self.ctrl_dict[ctrl_type]["tools_pub"].publish(coor_name)
                rospy.sleep(0.1)

            # 抓取过程
            # 增加延迟，避免抓取过程中工具坐标系没有改正过来
            time.sleep(0.5)

            v = v_.copy()
            v[2] += 0.03

            # 计算机器人预抓取位置
            if ctrl_type == 1:
                v[0] -= 0.03
            elif ctrl_type == 2:
                v[0] += 0.03

            self._movejp_plan(q_, v, ctrl_type, speed=arm_speed)
            if not self._wait_arm_planned(ctrl_type):
                rospy.logwarn(f"{arm_name}预抓取动作执行失败！")
                continue

            self._movejp_plan(q_, v_, ctrl_type, speed=arm_speed)
            if not self._wait_arm_planned(ctrl_type):
                rospy.logwarn(f"{arm_name}抓取动作执行失败！")
                continue

            self._grip_materiel("close", ctrl_type)
            rospy.sleep(3)
            self._movejp_plan(q_, v, ctrl_type, speed=arm_speed)
            if not self._wait_arm_planned(ctrl_type):
                rospy.logwarn(f"{arm_name}拿开动作执行失败！")
                # 放回物料
                self._movejp_plan(q_, v_, ctrl_type, speed=arm_speed)
                self._wait_arm_planned(ctrl_type)
                # 打开夹爪
                self._grip_materiel("open", ctrl_type, speed=arm_speed)
                rospy.sleep(3)
                continue

            # 过渡姿态
            self._movej_plan(self.ctrl_dict[ctrl_type]["joints"][1], ctrl_type, speed=arm_speed)
            self._wait_arm_planned(ctrl_type)

            # 放置动作中容易发生碰撞，需要加上线程锁，避免两个臂同时放置物料到物料框中
            with self.placement_lock:
                # 放置姿态
                self._movej_plan(self.ctrl_dict[ctrl_type]["joints"][2], ctrl_type, speed=arm_speed)
                self._wait_arm_planned(ctrl_type)

                self._grip_materiel("open", ctrl_type)
                rospy.sleep(3)
                # 识别姿态
                self._movej_plan(self.ctrl_dict[ctrl_type]["joints"][0], ctrl_type, speed=arm_speed)
                self._wait_arm_planned(ctrl_type)

            # 累加
            index += 1
        
        # 机械臂抓取动作完成
        self.actions_thread_event[ctrl_type] = True


    # 机器人步进运动
    def _base_step_plan(self, step: float, speed: float=0.1, 
                        use_avoid: bool=False, mode: int=StepControlGoal.STRAIGHT):
        
        # 创建一个目标对象
        goal = StepControlGoal()
        # 等待动作服务端可用
        if not self.base_step_client.wait_for_server(timeout=rospy.Duration(5.0)):
            rospy.logerr("Action server not available within timeout")
            return False
        
        # 设置机器人动作状态为执行
        goal.mode = 1
        # 是否躲避障碍物
        goal.useAvoid = use_avoid
        
        # 初始化控制数据压到任务列表：stepControl
        step_control = StepControl()
        step_control.data = step
        step_control.speed = speed
        step_control.executeMode = mode

        # 设置控制数据
        goal.stepControl.append(step_control)

        # 发送目标到action server
        self.base_step_client.send_goal(goal)

        # 等待action server处理完成
        self.base_step_client.wait_for_result()

        # 获取执行结果
        result = self.base_step_client.get_result()

        # 判断执行结果
        if result is None:
            rospy.logerr("Action server did not return a result")
            return False
        else:
            if result.result:
                rospy.loginfo("Move backward completed successfully")
                return True
            else:
                rospy.loginfo(f"Service state is: {self.base_step_client.get_state()}")
                rospy.logwarn("注意，旁边有障碍，请避让")
                time.sleep(5)
                return self._base_step_plan(step=step, speed=speed)

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

    # 打开相机
    def _set_camera_status(self, run: bool, camera_name: str):
        # 相机请求数据
        camera_req = CameraSettingRequest()
        camera_req.frame_id = camera_name
        camera_req.run = run
        while not rospy.is_shutdown():
            try:
                camera_result = self.camera_client(camera_req)
                if camera_result.result:
                    rospy.loginfo(camera_result.message)
                    return True
                else:
                    rospy.logerr(camera_result.message)
                    return False
            except Exception as err:
                rospy.loginfo(f"Failed to call camera service: {err}!")
            finally:
                time.sleep(0.2)
    
    # 得到相机识别结果
    def _get_aruco_detect_result(self, num: int=582, link_name: str="camera_middle",delay: float=10.0):
        # 读取头部相机识别结果
        req = HeadResultRequest()
        req.num = num
        req.delayed = delay
        req.link_name = link_name
        result = self.head_detect_client(req)
        return result
    
    # 得到相机识别结果
    def _get_material_detect_result(self, label: str, link_name: str, delay: float=10.0):
        # 读取头部相机识别结果
        req = HandResultRequest()
        req.label = label
        req.delayed = delay
        req.link_name = link_name
        result = self.hand_detect_client(req)
        return result

    # 处理物料识别结果
    def _analysis_materiel_detect_result(self, pose: Pose):
        # 旋转四元数
        q = [pose.orientation.w, 
             pose.orientation.x, 
             pose.orientation.y,
             pose.orientation.z]
        v = list()

        v.append(pose.position.x)
        v.append(pose.position.y)
        v.append(pose.position.z)
        
        return q, v

    # 处理AR码识别结果
    def _ar_tf_result(self, pose: List[Pose], vector: list):
        # 初始化pose_vector容器
        pose_vector = list()

        for index in range(len(pose)):

            if bool(index):
                sign = 1
            else:
                sign = -1
            
            # 获取姿态
            q = [pose[index].orientation.w,
                 pose[index].orientation.x,
                 pose[index].orientation.y,
                 pose[index].orientation.z]
            # 打印数据
            print("\n", "^" * 20)
            print("orientation: ", q)
            print("position: ", [pose[index].position.x, 
                                 pose[index].position.y, 
                                 pose[index].position.z])
        
            vector_ = list()
            # 将偏置付给容器
            vector_.append(sign * vector[0])
            vector_.append(vector[1])
            vector_.append(vector[2]) 
            vectored = self.calculator.transform_vector_by_quaternion(q, vector_)
            print(f"vectored{ index }: { vectored }")
            
            pose_ = [0.0] * 3
            pose_[0] = pose[index].position.x + vectored[0]
            pose_[1] = pose[index].position.y + vectored[1]
            pose_[2] = pose[index].position.z + vectored[2]

            pose_vector.append({
                "position": pose_, 
                "orientation": q
                })
        
        return pose_vector

    # 升降装置执行系统
    def _set_lift_height(self, height, speed: 50.0):
        msg = Lift_Height()
        msg.height = height
        msg.speed = speed
        self._left_is_plan_succeed = False
        self.lift_height_pub.publish(msg)

    # ros时间戳回调汉书
    def _agv_timestemp_listen(self, event):
        
        if self._woosh_task_is_succeed:
            self._wait_navigation_event.set()
            rospy.loginfo("Navigation was successful!")
        else:
            pass

    # ros时间戳回调汉书
    def _arm_timestemp_listen(self, arm_type: int):
        """
        机械臂规划ROS时间周期回调函数。

        Args:
            arm_type: int
        """
        if arm_type == 1 and self._left_is_plan_succeed:
            # 打开左臂
            self._wait_arm_event[arm_type].set()
            rospy.loginfo("Left arm was successful!")
        elif arm_type == 2 and self._right_is_plan_succeed:
            # 打开右臂
            self._wait_arm_event[arm_type].set()
            rospy.loginfo("Right arm was successful!")
        elif arm_type == 3 and self._right_is_plan_succeed and self._left_is_plan_succeed:
            # 打开左臂右臂
            self._wait_arm_event[arm_type].set()
            rospy.loginfo("Right and Left arms were successful!")
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
        self._woosh_task_is_succeed = False  # 避免底盘状态改变之后自己来不及置零，也避免执行过快忽略置零。
        
        res = self.navigation_point(req)
        return res.success
    
    # 导航等待
    def _navigation_wait(self, carry, timeout=120):

        # self._woosh_task_is_succeed = False  # 避免底盘状态改变之后自己来不及置零
        self._wait_navigation_event.clear()
        # 创建时间戳
        timer = rospy.Timer(rospy.Duration(0.5), self._agv_timestemp_listen, oneshot=False)
        if carry:
            if not self._wait_navigation_event.wait(timeout=timeout):
                rospy.logerr(f"Navigation execution timeout (greater than { timeout } seconds!)")
                timer.shutdown()
                return False
            else:
                rospy.loginfo(f"Navigation execution was successful!")
                timer.shutdown()
                return True
        else:
            rospy.logerr(f"Navigation execution command error!")
            timer.shutdown()
            return False

    # movej规划
    def _movej_plan(self, joints: list, type: int, speed: int=10):
        msg = MoveJ()
        
        for joint in joints:
            msg.joint.append(math.radians(joint))

        msg.speed = speed / 100.0
        msg.trajectory_connect = False
        if type == 1:
            self._left_is_plan_succeed = False
            self.left_movej_pub.publish(msg)
        elif type == 2:
            self._right_is_plan_succeed = False
            self.right_movej_pub.publish(msg)
        else:
            pass
    
    # movejp规划
    def _movejp_plan(self, orientation: list, pose: list, type: int, speed: int=10):
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
            self._left_is_plan_succeed = False
            self.left_movejp_pub.publish(msg)
        elif type == 2:
            self._right_is_plan_succeed = False
            self.right_movejp_pub.publish(msg)
        else:
            pass

    # movel规划
    def _movel_plan(self, pose: list, gesture: list, type: int, speed: int=10):
        msg = MoveL()
        msg.Pose.position.x = pose[0]
        msg.Pose.position.y = pose[1]
        msg.Pose.position.z = pose[2]
        if len(gesture) == 4:
            msg.Pose.orientation.x = gesture[0]
            msg.Pose.orientation.y = gesture[1]
            msg.Pose.orientation.z = gesture[2]
            msg.Pose.orientation.w = gesture[3]
        elif len(gesture) == 3:
            # 将欧拉角转换为四元数
            quaternion = tf.transformations.quaternion_from_euler(gesture[0], gesture[1], gesture[2])
            
            msg.Pose.orientation.x = quaternion[0]
            msg.Pose.orientation.y = quaternion[1]
            msg.Pose.orientation.z = quaternion[2]
            msg.Pose.orientation.w = quaternion[3]
        else:
            rospy.logerr("Movel-Plan's params are error!")
            return
        # 设置速度
        msg.speed = speed / 100.0
        msg.trajectory_connect = False
        if type == 1:
            # 切换工具坐标系至tip
            coor_name = ChangeTool_Name()
            coor_name.toolname = self.left_tool3
            for _ in range(3):
                self.set_left_tool_frame.publish(coor_name)
                rospy.sleep(0.1)
            time.sleep(0.1)
            self._left_is_plan_succeed = False
            self.left_movel_pub.publish(msg)
        elif type == 2:
            # 切换工具坐标系至tip
            coor_name = ChangeTool_Name()
            coor_name.toolname = self.right_tool3
            for _ in range(3):
                self.set_right_tool_frame.publish(coor_name)
                rospy.sleep(0.1)
            time.sleep(0.1)
            self._right_is_plan_succeed = False
            self.right_movel_pub.publish(msg)
        else:
            pass


if __name__=='__main__':
    rospy.init_node("body_handling_action_node")
    robot = Robot_Action()
    # 动作开始执行
    robot.action_thread.start()
    rospy.spin()