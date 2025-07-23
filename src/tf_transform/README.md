## **一.项目介绍**
该文件夹中共有三个功能包：
1. tf_calculate: 合成完整的tf坐标系，里面的文件都是设置tf坐标转换。
2. tf_demo: 测试tf坐标系各个坐标系之间的转换关系。
3. tf_msgs: 自定义tf坐标系合成过程中所以要的各种msg、action、srv文件。

## **二.代码结构**
    src/tf_transform
     │ README.md    <- 文件描述
     │
     ├─tf_calculate    <- 合成完整的tf坐标系
     │  │ CMakeLists.txt
     │  │ package.xml
     │  │
     │  ├─include
     │  │  └─tf_calculate    <- .h头文件目录
     │  ├─launch
     │  │    target2base_link.launch    <- launch启动文件
     │  │
     │  ├─scripts
     │  │    camera_trans.py    <- 相机手眼标定参数写入脚本，获取目标位置基于机械臂基坐标系的相对位姿。(python)
     │  │
     │  └─src
     │       camera_trans.cpp    <- 相机手眼标定参数写入脚本，获取目标位置基于机械臂基坐标系的相对位姿。(C++)
     │       target_trans.cpp    <- 获取目标物体基于相机坐标系的相对位姿。
     │
     ├─tf_demo    <- 测试tf坐标系各个坐标系之间的转换关系。
     │  │ CMakeLists.txt
     │  │ package.xml
     │  │
     │  ├─include
     │  │  └─tf_demo    <- .h头文件目录
     │  ├─launch
     │  ├─scripts
     │  │    tf_demo.py    <- tf坐标订阅测试脚本
     │  │
     │  └─src
     └─tf_msgs    -> 自定义tf坐标系合成过程中所以要的各种msg、action、srv文件。
        │  CMakeLists.txt
        │  package.xml
        │
        ├─action    <- 自定义动作消息类型
        │    Shoot.action
        │
        ├─include
        ├─msg    <- 自定义话题消息类型
        │    GraspPose.msg
        │    TargetResult.msg
        │
        └─srv    <- 自定义服务消息类型
             ControlArm.srv
             ControlArmMoveitPose.srv
             ControlArmMoveitTarget.srv
             ControlGripper.srv
             ControlLift.srv
             Target2Agv.srv
             Target2Base.srv
             TargetDetect.srv
             WaterAgv.srv

## **三.环境与依赖**
* ROS1版本：noetic
* Linux版本：Ubuntu20.04
* 系统架构：X86/ARM64

## **四.编译方法**
创建工作空间，例如：catkin_ws，将tf_transform文件夹复制到src下。首先编译rm_msgs、tf_msgs功能包，其次再进行全局编译：

```bash
cd ~/catkin_ws     # 进入~/catkin_ws目录
catkin build rm_msgs    # 先编译下消息类型包，避免编译整个工作空间时找不到自定义的头文件
catkin build        # 编译整个工作空间
```

## **五.运行指令**
- 1.启动整体ros功能包的launch

```bash
cd ~/catkin_ws
source devel/setup.bash    # 重新加载下环境变量

roslaunch tf_calculate target2base_link.launch;    # ros驱动启动文件
```

- 2.启动功能包功能的使用案例：

```bash
rosrun tf_demo tf_demo.py;    # 测试ddemo驱动文件
```