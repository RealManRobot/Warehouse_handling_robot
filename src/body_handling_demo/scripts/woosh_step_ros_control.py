#!/usr/bin/env python3
# -*- coding=UTF-8 -*-

import time
# ROS库
import rospy

from woosh_msgs.msg import StepControlAction, StepControlGoal, StepControl
import actionlib

class Robot_Woosh:

    def __init__(self):
        # 创建Action客户端
        # 创建一个SimpleActionClient，连接到/cmd_vel_control action server
        self.base_step_client = actionlib.SimpleActionClient('/cmd_vel_control', StepControlAction)

    # 机器人步进运动
    def base_step_plan(self, step: float, speed: float=0.1, 
                        use_avoid: bool=False, mode: int=StepControlGoal.ROTATE):
        
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
                rospy.logerr("Move backward failed")
                rospy.logwarn("注意，傍边有障碍，请避让")
                time.sleep(5)
                return self.base_step_plan(step=step, speed=speed)
            


if __name__=='__main__':
    rospy.init_node("woosh_step_ros_control")

    agv = Robot_Woosh()
    agv.base_step_plan(0.10)

    rospy.spin()