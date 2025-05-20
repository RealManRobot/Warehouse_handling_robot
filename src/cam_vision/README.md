## **一.项目介绍**
该文件夹中共有两个功能包，1.cam_demo功能包、2.cam_identify功能包。cam_demo功能包包含了底盘导航ros驱动的测试脚本cam_demo.py，目前预制点位名称仅有“A”,"B","C"和"D"。打开cam_demo与cam_identify之后，通过键盘输入的目标物标签值，从而获取该目标物相对于camera_link坐标系的位姿。

## **二.代码结构**
    src/cam_vision
     │  README.md    <- 文件描述
     │
     ├─cam_demo    <- 相机测试实例
     │  │
     │  │ CMakeLists.txt
     │  │ package.xml
     │  │
     │  ├─include
     │  │  └─cam_demo    <- .h头文件目录
     │  ├─launch
     │  │    cam_demo.launch    <- launch启动文件
     │  │
     │  ├─scripts
     │  │    cam_demo.py    <- 测试实例脚本
     │  │    cam_demo.sh    <- bash命令启动脚本
     │  │
     │  └─src    <- cpp源文件目录
     └─cam_identify    <- ros相机驱动实例
        │
        ├─cam_module    <- 该目录里面为驱动相机执行脚本
        │
        ├─launch
        │    cam_identify.launch    <- launch启动文件
        ├─files_pth    <- 模型权重
        │    10500_50.pt
        │    best.pt
        │    CDNet.pth
        │    shen.pt
        │
        ├─include
        │  └─cam_identify    <- .h头文件目录
        ├─launch
        │    cam_identify.launch    <- launch启动文件
        │
        ├─mp3    <- .mp3库，用以音频调用
        │    jiuwei.mp3    <- 就位语音
        │    lingqu.mp3    <- 领取语音
        │    nan_kaishi.mp3    <- 开始语音
        │    qianchongdian.mp3    <- 充电语音
        │    qudan.mp3    <- 取单语音
        │    songzhong.mp3    <- 正在送单
        │    start.mp3    <- 开始送单
        │
        ├─scripts
        │    cam_identify.py    <- 得到目标坐标系基于机械臂坐标系的相对关系
        │    custom_import.py    <- 配置相对目录的脚本
        │    test_mp3.py
        │
        └─src    <- cpp源文件

## **三.环境与依赖**
* ROS1版本：noetic
* Linux版本：Ubuntu20.04
* 系统架构：X86/ARM64

## **四.编译方法**
创建工作空间，例如：catkin_ws，将cam_vision文件夹复制到src下。首先编译rm_msgs功能包，其次再进行全局编译：

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

roslaunch cam_driver cam_identify.launch;    # ros驱动启动文件
```

- 2.启动功能包功能的使用案例：

```bash
roslaunch cam_demo cam_demo.launch;    # 测试ddemo驱动文件
```