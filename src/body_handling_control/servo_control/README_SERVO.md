
## **一.项目介绍**
舵机ros功能包，通过话题改变舵机角度和获取舵机状态

## **二.代码结构**
servo_control
    ├── README.md
    ├── servo_demo
    │   ├── CMakeLists.txt
    │   ├── include
    │   │   └── servo_demo
    │   ├── package.xml
    │   ├── scripts
    │   │   └── servo_control_demo.py 测试舵机ros的demo，通过发布和订阅话题来改变舵机角度和获取舵机状态
    │   └── src
    └── servo_ros
        ├── CMakeLists.txt
        ├── launch
        |   └── servo_start.launch 舵机启动的launch
        ├── msg 自定义舵机消息
        │   ├── Servo_angle.msg
        │   ├── Servo_GetAngle.msg
        │   └── Servo_Move.msg
        ├── package.xml
        └── src
            └── servo_controller.cpp  舵机ros包主要逻辑代码


## **三.编译方法**

- 创建 文件夹
    - mkdir -p ~/catkin_ws/src
- 将 servo_control文件夹放入工作空间catkin_ws/src/中
- 编译ros包

    - cd ~/catkin_ws
    - catkin build 

## **四.运行指令**
- 1.启动舵机ros功能包的launch
    '''
    cd ~/catkin_ws
    source devel/setup.bash
    rosluanch servo_ros servo_start.launch
    '''


- 2.启动功能包功能的使用案例demo：
    '''
    rosrun servo_demo servo_control_demo.py
    '''