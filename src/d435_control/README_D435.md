## **一.项目介绍**
本功能包是对d435相机的封装，通过话题可以得到颜色帧、深度帧和相应像素点的深度值。

## **二.代码结构**


d435_control
    ├── d435_demo 获取相机颜色帧、深度帧和相应像素点的深度值的 demo 示例包
    │   ├── CMakeLists.txt
    │   ├── include
    │   │   └── d435_demo
    │   ├── package.xml
    │   ├── scripts
    │   │   ├── get_frame_demo.py 获取颜色帧/深度帧 demo 示例
    │   │   └── get_pixel_depth_demo.py 获取指定像素点的深度值 demo 示例
    │   └── src
    ├── d435_ros 封装的D435功能包
    │   ├── CMakeLists.txt
    │   ├── include
    │   │   └── d435_ros
    │   ├── launch
    │   │   └── camera_launch.launch d435相机启动launch文件
    │   ├── msg
    │   │   └── Pixel.msg 自定义消息类型（像素点）
    │   ├── package.xml
    │   ├── scripts
    │   │   └── frame_show.py 相机ros包的主要逻辑代码
    │   └── src 
    └── README.md #功能包说明文件



## **三.编译方法**

- 创建 文件夹
    - mkdir -p ~/catkin_ws/src
- 将 d435_camera文件夹放入工作空间catkin_ws/src/中
- 编译ros包

    - cd ~/catkin_ws
    - catkin build 

    


## **四.运行指令**

- 1.启动相机ros功能包的launch

    1. cd ~/catkin_ws
    2. source devel/setup.bash
    3. roslaunch d435_ros camera_start.launch

- 2.启动demo示例：

    - 获取颜色帧demo 示例

        '''
        rosrun d435_demo get_frame_demo.py 1

    - 获取深度帧demo 示例

        '''
        rosrun d435_demo get_frame_demo.py 2
     
        '''

    - 获取（100，200）这个像素点深度 demo 示例（需要将 100 200 添加到运行命令后面）

        '''
        
        rosrun d435_demo get_pixel_depth_demo.py 100 200
        
        '''
