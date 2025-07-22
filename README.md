# 仓储搬运机器人demo实例

## 一、项目概述

### 1.1 项目简介

具身双臂机器人仓储搬运系统是一个专为展会演示设计的自动化解决方案，展示了双臂机器人在仓储物流场景中的协调作业能力。该系统通过视觉识别、精准定位和双臂协同控制，实现了物料框的自动抓取、运输和摆放功能，体现了现代工业机器人在智能制造领域的应用潜力。

**注：**如果确定整个demo已经配置完毕之后，换了不同的场景之后，只需要对底盘进行重新配置即可。

<img src=".\images\robot.jpg" alt="具身双臂机器人" style="zoom:15%;" />

### 1.2 核心功能

* **双臂协同作业**：左右机械臂可独立或协同完成物料搬运任务

* **视觉识别系统**：

  - 基于Aruco码的物料框定位

  - 多相机（左臂、右臂、头部相机）目标识别

* **自主导航**：

  - 基于Woosh底盘的SLAM建图与导航

  - 精确点位到达控制

* **智能充电**：低电量自主充电功能

* **多模式控制**：支持自动模式和手动调试模式

### 1.3 应用场景

该demo特别适合以下场景：

- 工业展会技术演示
- 仓储物流自动化展示
- 智能制造教学示范
- 机器人协同作业研究

### 1.4 项目成果

* 完整的软硬件集成解决方案

* 详细的布展流程和技术文档

* 可复用的ROS功能包

* 稳定的演示系统（支持24小时连续运行）

* 开机自启动配置方案

该项目展示了双臂机器人在复杂任务中的协调控制能力，为工业自动化应用提供了实用的技术参考。系统设计注重实用性和可靠性，特别适合需要长时间稳定运行的展会环境。

### 1.5 更新日志

| 更新日期 |            更新内容            | 版本号 |
| :------: | :----------------------------: | :----: |
| 2025/4/8 | 仓储搬运机器人demo初始版本发布 | v1.0.0 |

## 二、布展环境准备

布展环境占地面积大致为 4\*4 m²，其中货架占地约为 0.5\*1.5 m²，桌子占地面积约为 0.6\*1.2 m²，机器人本体占地面积约为 0.4\*0.7 m²。

* 货架共分为4层，按照卡口位置从上到下进行定位如下：

| 参数名称 | 卡口位置 | 对应升降机高度 |
| :------: | :------: | :------------: |
|  floor1  |    7     |     0.105      |
|  floor2  |    15    |     0.585      |
|  floor3  |    23    |     1.080      |
|  floor4  |    31    |       ∞        |

* 取货拿货时升降机高度：

|  参数名称  |     状态备注     | 对应升降机高度 |
| :--------: | :--------------: | :------------: |
| lift_send  | 机器人运送物料时 |     0.645      |
| lift_table | 机器人抓取物料时 |     0.305      |

**注：**需要按照中间层板上卡扣的下方作为准则进行定位（第3，4层货架由于目前升降机高度不够导致无法到达）。

<img src=".\images\counter.jpg" alt="货架定位方式" style="zoom:15%;" />

* 桌子摆放的时候要确保周围有1.5m的膨胀范围，确保机器人有足够的空间规划到抓取物料的点位。

![table](.\images\table.jpg)

* 物料框要放置在货架每层板子的中间，Aruco码要朝向外边，物料框尽量放置在货架边缘，确保机械臂有足够的作业空间。另外，Aruco码要粘贴至物料框正中间。

![box](.\images\box.jpg)

* 物料框要放置于每一层的1/3处和2/3处，避免机械臂规划过程中碰撞到货架两侧。

![boxes](.\images\boxes.jpg)

* 充电桩放置的时候应确保其前方至少2~3m内无障碍物，充电点的位置为充电桩前方1~1.5m，2~3m的空间将会方便机器人充电对接过程的稳定执行。

<img src=".\images\change.jpg" alt="change" style="zoom:66%;" />

## 三、硬件环境说明

### 3.1 设备硬件结构

![robot_body](.\images\robot_body.jpg)

### 3.2 硬件接线说明

![network](.\images\network.png)

**注：**由于用户主机2和底盘主控是通过两者的WIFI网卡直接连接某个相同的WIFI上进行连接的，所以用户主机2并不能通过2访问机器人的机械臂。

### 3.3 硬件配置说明

#### 3.3.1 机械臂配置

* **左右臂ip配置**

众所周知，机械臂默认IP为`192.168.1.18`。由于局域网IP要求为`169.254.128.xxx`网段，则设置左臂`L_IP：169.254.128.18`，右臂`R_IP：169.254.128.19`即可。进入示教器的IP配置过程为：`系统配置`=>`通讯配置`=>`IP模式配置`=>`目标IP：L_IP/R_IP`，最后设置网关`169.254.128.1`和掩码`255.255.255.0`并点击设置即可。

<img src=".\images\Net_Setting.png" alt="ip配置" style="zoom:50%;" />

* **左右机械臂零位配置**

左右臂处于零位的时候要确保机械臂末端相机朝上，且机械臂末端夹爪平行于相机，末端托手与相机夹角为90°，具体效果图如下所示：

<img src=".\images\zero.jpg" alt="机械臂零位" style="zoom:10%;" />

| ![右臂](.\images\right_grip.jpg) | ![左臂](.\images\left_grip.jpg) |
| :------------------------------: | :-----------------------------: |
|          右臂末端实物图          |         左臂末端实物图          |

* **左右臂工具坐标系设置**

为了适应于demo，需要分别设置机械臂末端法兰工具坐标系(`tip_*`)、机械臂夹爪工具坐标系(`grip_*`)、机械臂托手工具坐标系(`nip_*`)。配置过程是将[左臂工具坐标系txt参数脚本](./source/tool_coordinate_system_left.txt)和[右臂工具坐标系txt参数脚本](./source/tool_coordinate_system_right.txt)通过示教器上传即可：`机械臂配置`=>`工具坐标系标定`=>`坐标系文件导入（右边中间的高亮蓝色按钮）`。

<img src=".\images\tool.png" alt="tool" style="zoom:50%;" />

* **拓展关节配置**

在demo启动之前必须确保机器人的拓展关节（末端控制与升降机控制）已经成功连接并使能。其中，升降机控制部分要确保减速比系数为0.0066、最小位置为-10mm、最大位置为820mm，末端控制部分要确保电压为24V并成功连接即可。

#### 3.3.2 底盘配置

**注：**机器人更换不同场景时必然需要重新进行底盘配置操作。

1. **打开软件**

![打开软件](.\images\0.png)

2. **连接机器人（确保机器人wifi是打开的，并且保证已连接wifi）**

   * 将旋钮旋转到竖直向上的位置（切换到手动模式），同时长按暂停继续的按钮、复位按钮直到双灯闪烁，并且有外放类似于wifi已经打开的声音。

   <img src=".\images\6.png" alt="开启wifi" style="zoom: 15%;" />

   * wifi连接机器人：wifi名：`WOOSH_000000Z`，密码：`woosh888` 。

   <img src=".\images\1.png" alt="点击连接机器人" style="zoom:50%;" />

   <img src=".\images\2.png" alt="连接" style="zoom:50%;" />

3. **开始建图**

   <img src=".\images\3.png" alt="点击地图" style="zoom:50%;" />

   <img src=".\images\4.png" alt="添加场景" style="zoom:50%;" />

   <img src=".\images\5.png" alt="部署模式" style="zoom:50%;" />

   <img src=".\images\7.png" alt="开始建图" style="zoom:50%;" />

   建图成功保存即可。

4. **采导航点位**

   根据demo要求，机器人默认将两个物料框填满。货架共分为四层，机器人可作业范围为第一层和第二层。

   与此同时，货架共分为两列，每个抓取物料框的点位都和货架列数有关。此外，充电桩前还需设置一个1~1.5m左右的充电点位。

   * 首先打开软件Woosh Design：

     ![图标](.\images\8.png)

   * 点击设置下拉框中的连接按钮选择连接的机器人。

     <img src=".\images\9.png" alt="设置" style="zoom:50%;" />

     <img src=".\images\10.png" alt="连接" style="zoom:50%;" />

   * 之后点击窗口左侧中间的部署按钮：

     <img src=".\images\11.png" alt="部署点位" style="zoom:50%;" />

   * 打开场景库时，先点击左下角场景库按钮，等待场景获取成功之后将点击机器人场景，之后打开正在使用的场景即可。

     <img src=".\images\12.png" alt="部署点位" style="zoom:50%;" />

   * 选择机器人ID(一般是10001):

     <img src=".\images\13.png" alt="部署点位" style="zoom:50%;" />

   * 首先，鼠标右键点击储位按钮，会显示储位类型，选择储位类型之后（本demo中只用到了充电桩和储位这两种类型），点击自动部署点位。之后，在右边的文本框中输入相应数据即可。最后，鼠标左键点击保存按钮即可创建好部署的点位。

     **注**：到达类型一定要是**精确到达**，不然点位误差会过大。在点击过保存按钮之后一定要点击**上传**按钮，不然无法创建成功。

     <img src=".\images\14.png" alt="部署点位" style="zoom:50%;" />

5. **点位名称规则说明**

|      点位类型      |                         点位名称规则                         | demo点位名称  |
| :----------------: | :----------------------------------------------------------: | :-----------: |
|      充电点位      |             不用改变，目前充电桩确定只有一个点位             |      120      |
|  空物料框抓取点位  | 站在货架正前方，从左到右数，1，2列<br/>（机器人橙色框距离货架大概300mm） |   1、2、...   |
| 货架前调整高度点位 | 空物料框抓取点位正后方，从左到右数，1，2列<br/>（在对应的空物料框抓取点位名称后加一个0）<br />（橙色框距离货架≥500mm） |  10、20、...  |
|    抓取物料点位    | 名称前两个字符为10，后根据物料排放顺序在后面加上对应数字<br />（机器胸部距离物料桌150mm） | 101、102、... |

<img src=".\images\16.png" alt="image-20250401162521780" style="zoom:66%;" />

## 四、软件环境说明

### 4.1 功能包结构

```tree
# tree -P "*.txt|*.py|*.cpp|*.msg|*.srv|*.action|*.launch|*.yaml|*.xml" > tree_special_files.txt

./src
├── body_handling_control  => 放置机械臂、舵机驱动功能包的文件夹
│   ├── arm_control
│   │   ├── arm_driver  => 机械臂驱动功能包
│   │   │   ├── CMakeLists.txt
│   │   │   ├── launch
│   │   │   │   ├── dual_arm_65_driver.launch
│   │   │   │   └── dual_arm_75_driver.launch
│   │   │   ├── package.xml
│   │   │   └── src
│   │   │       └── arm_driver.cpp  => 驱动源代码
│   │   └── dual_arm_msgs  => 机械臂消息类型
│   │       ├── CMakeLists.txt
│   │       ├── msg
│   │       │   ├── Arm_Analog_Output.msg
│   │       │   ├── Arm_Current_State copy.msg
│   │       │   ├── Arm_Current_State.msg
│   │       │   ├── Arm_Digital_Output.msg
│   │       │   ├── Arm_IO_State.msg
│   │       │   ├── Arm_Joint_Speed_Max.msg
│   │       │   ├── Arm_Pose_Euler.msg
│   │       │   ├── Arm_Software_Version.msg
│   │       │   ├── ArmState.msg
│   │       │   ├── Cabinet.msg
│   │       │   ├── CarteFdPose.msg
│   │       │   ├── CartePos.msg
│   │       │   ├── ChangeTool_Name.msg
│   │       │   ├── ChangeTool_State.msg
│   │       │   ├── ChangeWorkFrame_Name.msg
│   │       │   ├── ChangeWorkFrame_State.msg
│   │       │   ├── Force_Position_Move_Joint.msg
│   │       │   ├── Force_Position_Move_Pose.msg
│   │       │   ├── Force_Position_State.msg
│   │       │   ├── GetArmState_Command copy.msg
│   │       │   ├── GetArmState_Command.msg
│   │       │   ├── Gripper_Pick.msg
│   │       │   ├── Gripper_Set.msg
│   │       │   ├── Hand_Angle.msg
│   │       │   ├── Hand_Force.msg
│   │       │   ├── Hand_Posture.msg
│   │       │   ├── Hand_Seq.msg
│   │       │   ├── Hand_Speed.msg
│   │       │   ├── IO_Update.msg
│   │       │   ├── Joint_Current.msg
│   │       │   ├── Joint_Enable.msg
│   │       │   ├── Joint_Error_Code.msg
│   │       │   ├── Joint_Max_Speed.msg
│   │       │   ├── JointPos.msg
│   │       │   ├── Joint_Step.msg
│   │       │   ├── Joint_Teach.msg
│   │       │   ├── Lift_Height.msg
│   │       │   ├── Lift_Speed.msg
│   │       │   ├── LiftState.msg
│   │       │   ├── Manual_Set_Force_Pose.msg
│   │       │   ├── MoveC.msg
│   │       │   ├── MoveJ.msg
│   │       │   ├── MoveJ_P.msg
│   │       │   ├── MoveJ_PO.msg
│   │       │   ├── MoveL.msg
│   │       │   ├── Ort_Teach.msg
│   │       │   ├── Plan_State.msg
│   │       │   ├── Pos_Teach.msg
│   │       │   ├── Servo_GetAngle.msg
│   │       │   ├── Servo_Move.msg
│   │       │   ├── Set_Force_Position.msg
│   │       │   ├── Set_Realtime_Push.msg
│   │       │   ├── Six_Force.msg
│   │       │   ├── Socket_Command.msg
│   │       │   ├── Start_Multi_Drag_Teach.msg
│   │       │   ├── Stop.msg
│   │       │   ├── Stop_Teach.msg
│   │       │   ├── Tool_Analog_Output.msg
│   │       │   ├── Tool_Digital_Output.msg
│   │       │   ├── Tool_IO_State.msg
│   │       │   └── Turtle_Driver.msg
│   │       └── package.xml
│   └── servo_control
│       ├── servo_demo  => 舵机驱动功能包
│       │   ├── CMakeLists.txt
│       │   ├── package.xml
│       │   └── scripts
│       │       └── servo_control_demo.py  => 舵机demo测试脚本
│       └── servo_ros  => 舵机驱动脚本
│           ├── CMakeLists.txt
│           ├── launch
│           │   └── servo_start.launch
│           ├── msg
│           │   ├── ServoAngle.msg
│           │   └── ServoMove.msg
│           ├── package.xml
│           └── src
│               └── servo_controller.cpp  => 机械臂驱动源代码
├── body_handling_demo
│   ├── CMakeLists.txt
│   ├── include
│   │   └── body_handling_demo
│   ├── launch
│   │   ├── body_handling_action.launch  => 整体demo启动脚本 *
│   │   ├── body_handling_change.launch  => 充电demo启动脚本
│   │   ├── body_handling_detect.launch
│   │   ├── body_handling_double_gripper.launch  => 双臂抓取demo启动脚本
│   │   └── body_handling_driver.launch  => 整体驱动启动脚本 *
│   ├── package.xml
│   ├── scripts
│   │   ├── body_handling_action.py  => 整体demo脚本
│   │   ├── body_handling_change.py  => 充电功能验证demo脚本
│   │   ├── calculate_position.py  => 坐标系转换模块
│   │   ├── cam_double_gripper_new.py  => 双臂抓取demo脚本，最新
│   │   ├── cam_double_gripper.py
│   │   ├── realsense_double_gripper.py  => 基于realsense的ros接口的双臂抓取demo脚本
│   │   ├── robot_detect.py
│   │   ├── test.py
│   │   └── woosh_api_test.py  => woosh语音模块测试脚本
│   └── src
│       └── test.cpp
├── cam_vision  => 放置相机识别功能包的文件夹
│   ├── cam_demo
│   ├── cam_identify  => 相机识别功能包
│   │   ├── CMakeLists.txt
│   │   ├── files_pth  => 模型权重文件保存路径
│   │   ├── include
│   │   │   └── cam_identify
│   │   ├── launch
│   │   │   ├── cam_identify.launch  => 通过python内置的realsense库驱动相机获取数据的识别脚本
│   │   │   └── realsense_identify.launch  => 通过realsense官方ros库驱动相机获取数据的识别脚本
│   │   ├── package.xml
│   │   ├── scripts
│   │   │   ├── cam_identify.py  => 识别脚本
│   │   │   ├── custom_import.py
│   │   │   ├── new_retail_identify.py
│   │   │   ├── realsense_identify.py
│   │   │   └── test.py
│   │   └── src
│   └── cam_msgs  => 相机识别消息类型
│       ├── CMakeLists.txt
│       ├── include
│       │   └── cam_msgs
│       ├── msg
│       │   └── Cabinet.msg
│       ├── package.xml
│       ├── src
│       └── srv
│           ├── DetectResult.srv
│           ├── HandResult.srv  => 双臂相机识别服务
│           └── HeadResult.srv  => 头部相机识别服务
├── d435_control  => 内置python的realsense接口打开相机的功能包
│   ├── d435_demo
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── scripts
│   │       ├── get_frame_demo.py
│   │       └── get_pixel_depth_demo.py
│   └── d435_ros
│       ├── CMakeLists.txt
│       ├── launch
│       │   └── camera_start.launch
│       ├── msg
│       │   └── RealsenseImage.msg
│       ├── package.xml
│       ├── scripts
│       │   └── camera_service.py
│       └── srv
│           ├── CameraSetting.srv
│           └── PixelToCoordinate.srv
├── tf_transform  => tf坐标设置与转化功能包
│   ├── tf_calculate
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   └── tf_calculate
│   │   ├── launch
│   │   │   ├── tfcoordinate_tree_copy.launch
│   │   │   └── tfcoordinate_tree.launch  => 坐标树启动脚本
│   │   ├── package.xml
│   │   ├── rviz
│   │   ├── scripts
│   │   │   └── tf_get.py
│   │   └── src
│   │       └── rm_arm_trans.cpp
│   ├── tf_demo  => tf坐标测试历程功能包
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   └── tf_demo
│   │   ├── launch
│   │   ├── package.xml
│   │   ├── scripts
│   │   │   └── tf_demo.py
│   │   └── src
│   └── tf_msgs  => tf坐标消息类型功能包（略）
│       ├── action
│       │   └── Shoot.action
│       ├── CMakeLists.txt
│       ├── include
│       ├── msg
│       │   ├── GraspPose.msg
│       │   └── TargetResult.msg
│       ├── package.xml
│       └── srv
│           ├── ControlArmMoveitPose.srv
│           ├── ControlArmMoveitTarget.srv
│           ├── ControlArm.srv
│           ├── ControlGripper.srv
│           ├── ControlLift.srv
│           ├── Ldle_Action.srv
│           ├── Target2Agv.srv
│           ├── Target2Base.srv
│           ├── TargetDetect.srv
│           └── WaterAgv.srv
└── tree_special_files.txt  => 文件结构树
```

### 4.2 软件基本环境

#### 4.2.1 系统、环境版本

* 系统版本：Ubuntu20.04
* 系统架构：arm64
* Python版本：Python3.8
* ROS版本：ros1-noetic

#### 4.2.2 Python基本环境

主控在安装过ros环境之后，可通过安装[requirements.txt](./source/requirements.txt)脚本中的Python库对Python环境进行补全，执行命令如下：

```bash
pip3 install -r requirements.txt -i https://pypi.doubanio.com/simple/
```

### 4.3 参数配置

* **body_handling_driver.launch内置参数**：`.src\body_handling_demo\launch\body_handling_driver.launch`

```html
<arg name="open_base" value="Base" />    <!-- 使用的机械臂全局坐标系名称，demo为Base --> 
<arg name="display" value="true" />    <!-- 是否打开rviz可视化 -->
```

* **body_handling_action.launch内置参数**：`.src\body_handling_demo\launch\body_handling_action.launch`

```html
<?xml version="1.0"?>
<launch>

    <node pkg="body_handling_demo" type="body_handling_action.py" name="body_handling_action" output="screen" >
        <!-- 物料框位置 -->
        <param name="cargo_box_col" value="1" type="int"/>
        <param name="cargo_box_row" value="2" type="int"/>
        <param name="cargo_box_num" value="1" type="int"/>
        
        <!-- 物料的位置 -->
        <rosparam param="cargoes_name">["JiaoHuanJi", "NSK", "AnNiu", "YiHeDa"]</rosparam>

        <!-- 层数->升降机高度 -->
        <param name="floor1" value="0.105" type="double"/>
        <param name="floor2" value="0.585" type="double"/>
        <param name="floor3" value="1.080" type="double"/>
        <param name="floor4" value="9.999" type="double"/>

        <!-- 固定位置->升降机高度 -->
        <param name="lift_send" value="0.645" type="double"/>
        <param name="lift_table" value="0.305" type="double"/>

        <!-- 速度控制 -->
        <param name="lift_speed" value="80" type="int"/>
        <param name="arms_speed" value="50" type="int"/>
        
        <!-- 工具坐标系名称 -->
        <param name="left_tool1" value="grip_left" type="string"/>
        <param name="left_tool2" value="nip_left" type="string"/>
        <param name="left_tool3" value="tip_left" type="string"/>
        <param name="right_tool1" value="grip_right" type="string"/>
        <param name="right_tool2" value="nip_right" type="string"/>
        <param name="right_tool3" value="tip_right" type="string"/>

        <!-- 设置自动模式的持续时间 s-->
        <param name="auto_mode_time" value="86400" type="int"/>

        <!-- 设置手动模式的持续时间 s-->
        <param name="manual_mode_time" value="86400" type="int"/>

        <!-- 设置自主充电点位 -->
        <rosparam param="charge_range">[20, 90]</rosparam>
        <param name="charge_pose" value="120" type="string"/>

        <!-- 设置头部舵机扭动角度 -->
        <rosparam param="servo_motor">[360, 500]</rosparam>
        <rosparam param="servo_tolerant">[3, 3]</rosparam> 

        <!-- 是否进行语音播报 -->
        <param name="talk_flag" value="true" type="bool"/>

        <!-- 预设置左右臂位置数据 -->
        <rosparam command="load" file="$(find body_handling_demo)/config/joints.yaml"/>
        <rosparam command="load" file="$(find body_handling_demo)/config/poses.yaml"/>

    </node>

</launch>
```

* **poses.yaml内置参数**：`.\src\body_handling_demo\config\poses.yaml`

```yaml
left_arm_poses:
  - [[0.07492, -0.45138, 0.14876], [1.577, -0.757, 0]]    # 左臂伸进货架
  - [[0.12100, -0.14400, 0.07963], [-1.57, -0.786, 3.141]]    # 左臂收回
right_arm_poses:
  - [[-0.07492, -0.45138, 0.14876], [-1.577, -0.757, 3.147]]    # 右臂伸进货架
  - [[-0.12100, -0.14400, 0.07963], [1.57, -0.786, 0]]    # 右臂收回
```

* **joints.yaml内置参数**：`.\src\body_handling_demo\config\joints.yaml`

```yaml
left_arm_joints:
  - [0, -90, -130, 90, 90, 85]    # 左臂收紧状态
  - [-78.068, -77.349, -93.606, 16.438, 83.680, -47.068]    # 左臂从货架上收缩
  - [-87.761, -70.033, -92.774, 6.623, 75.427, -43.667]    # 左臂放置物料框状态
  - [-75.226, -65.018, -90.613, 15.552, 66.080, -53.004]    # 左臂从物料框拿开
  - [-111.321, -7.430, -90.332, 45.072, -98.473, -25.041]    # 左臂识别目标物姿态
  - [-120.000, -10.000, -75.000, 45.000, -110.000, -20.000]    # 左臂抓取过渡姿态
  - [-121.120, -76.013, -36.879, 39.977, -75.353, -14.467]    # 左臂放置物料状态
  - [-79.906, -73.633, -127.319, 15.287, 111.774, -37.600]    # 左臂收起物料框
  - [-86.796, -70.679, -93.996, 6.333, 77.678, -43.493]    # 左臂将物料框放置到货架上

right_arm_joints:
  - [0, 90, 130, -90, -90, -85]    # 右臂收紧状态
  - [73.843, 78.015, 92.094, -15.814, -80.145, 49.309]    # 右臂从货架上收缩
  - [81.654, 71.927, 89.154, -8.213, -70.918, 49.307]    # 右臂放置物料框状态
  - [75.226, 65.018, 90.613, -15.552, -66.080, 53.004]    # 右臂从物料框拿开
  - [111.321, 7.430, 90.332, -45.072, 98.473, 25.041]    # 右臂识别目标物
  - [120.000, 10.000, 75.000, -45.000, 110.000, 20.000]    # 右臂抓取过渡姿态
  - [121.120, 76.013, 36.879, -39.977, 75.353, 14.467]    # 右臂放置物料状态
  - [74.952, 72.032, 127.261, -15.250, -108.318, 41.667]    # 左臂收起物料框
  - [81.651, 71.930, 89.174, -10.310, -69.756, 48.190]    # 右臂将物料框放置到货架上
```

* **camera_service.py参数**：`.\src\d435_control\d435_ros\scripts\camera_service.py`

```python
# 第12行
# 相机名称字典：相机名称：序列号
CAMERA_NAME = {
    "camera_left": '152122078151',
    "camera_right": '152222072647',
    "camera_middle": '052522070156'
}
```

首先打开新终端，之后执行`realsense-viewer`命令如下：

![realsense_viewer](.\images\realsense.png)

其次若最后一句输出显示的是`Found 5 Realsense devices`这样的字段则表示所有相机打开成功。启动命令之后会出现如下图所示的界面，只需通过点击`Add Source`添加相应的realsense设备便可得到状态栏，之后通过将`RGB Camera`切换至`on`便可得到相机2D/3D数据，观察相机数据确定是哪个相机。

![相机](.\images\camera.png)

最后，打开`info` 按键并将对应的`Serial Number`数据修改到camera_service.py里面对应的的参数即可。

* **相机标定结果**：`.\src\tf_transform\tf_calculate\src\rm_arm_trans.cpp`

```c++
// 第126行
int main(int argc, char **argv)
{
    // 初始化节点
    ros::init(argc, argv, "arm_state_publisher");
    ros::NodeHandle n("~");
    Listen_Arm_State arm_state_publisher(n);

    // 设置发布频率
    ros::Rate loop_rate(10);

    while (ros::ok())
    {  
        //创建tf的广播器
        static tf::TransformBroadcaster l_top_camera, r_top_camera, m_top_camera;
        // 初始化左臂tf数据，左臂相机标定
        tf::Transform camera_transform_l;
        camera_transform_l.setOrigin(tf::Vector3(0.083744, -0.040517, 0.009167));  // (x, y, z)
        tf::Quaternion quater_l;
        quater_l.setW(0.7057);
        quater_l.setX(-0.0124);
        quater_l.setY(-0.0082);
        quater_l.setZ(0.7083);
        camera_transform_l.setRotation(quater_l);

        // 广播flange与base之间的tf数据
        l_top_camera.sendTransform(tf::StampedTransform(camera_transform_l, ros::Time::now(), 
                                   arm_state_publisher.left_top_name, arm_state_publisher.left_camera_name));
        
        // 初始化右臂tf数据，右臂相机标定
        tf::Transform camera_transform_r;
        camera_transform_r.setOrigin(tf::Vector3(0.083744, -0.043517, 0.009167));  // (x, y, z)
        tf::Quaternion quater_r;
        quater_r.setW(0.7057);
        quater_r.setX(-0.0124);
        quater_r.setY(-0.0082);
        quater_r.setZ(0.7083);
        camera_transform_r.setRotation(quater_r);       
        
        r_top_camera.sendTransform(tf::StampedTransform(camera_transform_r, ros::Time::now(), 
                                   arm_state_publisher.right_top_name, arm_state_publisher.right_camera_name));
        
        /* 头部相机标定数据 */
        // 初始化头部tf数据, 左头部相机标定
        tf::Transform camera_transform_midl;

        camera_transform_midl.setOrigin(tf::Vector3(-0.160368, -0.091956, 0.114612));  // (x, y, z)
        tf::Quaternion quater_midl;
        quater_midl.setW(0.32180899);
        quater_midl.setX(0.22992885);
        quater_midl.setY(0.79605872);
        quater_midl.setZ(-0.4581072);
        camera_transform_midl.setRotation(quater_midl);

        // 广播flange与base之间的tf数据
        m_top_camera.sendTransform(tf::StampedTransform(camera_transform_midl, ros::Time::now(),
                                   arm_state_publisher.left_base_name, arm_state_publisher.midl_camera_name));

        // 初始化头部tf数据, 右头部相机标定
        tf::Transform camera_transform_midr;

        camera_transform_midr.setOrigin(tf::Vector3(0.198955, -0.109685, 0.068842));
        tf::Quaternion quater_midr;
        quater_midr.setW(-0.35264227);
        quater_midr.setX(-0.17364313);
        quater_midr.setY(0.80492861);
        quater_midr.setZ(-0.44450133);
        camera_transform_midr.setRotation(quater_midr);

        // 广播flange与base之间的tf数据
        m_top_camera.sendTransform(tf::StampedTransform(camera_transform_midr, ros::Time::now(),
                                   arm_state_publisher.right_base_name, arm_state_publisher.midr_camera_name));

        // 按照设定的频率循环
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
```

## 五、demo模块功能测试

### 5.1 功能测试

* **进入工作空间**

```bash
cd <工作空间>  # 进入工作空间

source devel/setup.bash  # 刷新环境变量
```

* **启动ros驱动节点**

```bash
roslaunch body_handling_demo body_handling_driver.launch  # 启动各个硬件的控制节点
```

* **测试各模块功能**

  * 测试抓取

  ```bash
  roslaunch body_handling_demo body_handling_double_gripper.launch
  ```

  * 测试充电

  ```bash
  roslaunch body_handling_demo body_handling_change.launch
  ```

  * 测试整体任务

  ```bash
  roslaunch body_handling_demo body_handling_action.launch  # 启动整体
  ```

**注：**上述三个脚本不能同时启动。

### 5.2 代码实例

* **body_handling_action.py的执行函数**

```python
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
```

* **calculate_position.py坐标转换程序**

```Python
#!/usr/bin/env python3
# -*- coding=UTF-8 -*-

import numpy as np


class Calculate_Position:
    
    def __init__(self):
        pass

    # 四元数相乘，表示先进行q2表示的旋转再进行q1表示的旋转。不符合交换定律
    def quaternion_multiply(self, q1, q2):
        '''
        返回四元数的实部（标量部分）表示旋转后的“方向”或“角度”,但由于四元数的旋转表示不依赖于标量部分,所以通常我们关注的是虚数部分w。
        返回四元数的虚数部分(x,y,z)表示旋转轴在旋转后的新的坐标系中的分量。
        '''
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        # 四元数相乘计算矩阵
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 + y1*w2 + z1*x2 - x1*z2,
            w1*z2 + z1*w2 + x1*y2 - y1*x2
        ])

    # 求出四元数共轭矩阵（表示四元数的逆运算），w实部轴不变，虚部反方向旋转
    def conjugate_quaternion(self, q):
        return np.array([q[0], -q[1], -q[2], -q[3]])

    # 将向量v由四元数q表示的旋转变换
    def transform_vector_by_quaternion(self, q, v):
        # 求出四元数q的逆运算
        q_conjugate = self.conjugate_quaternion(q)
        # print(q_conjugate)

        # 四元数乘法可以用来表示和执行三维空间中的旋转。当我们需要将一个向量绕某个轴旋转时，
        # 我们可以将这个向量表示为一个纯四元数，然后使用四元数乘法来应用旋转。
        v_quaternion = np.array([0, v[0], v[1], v[2]])

        # 四元数乘法，相当于在三维空间中将向量 v 绕由四元数 q 定义的轴旋转指定的角度，然后将结果转换回原始坐标系。
        v_transformed = self.quaternion_multiply(self.quaternion_multiply(q, v_quaternion), q_conjugate)
        # print(v_transformed)
        return v_transformed[1:]

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return np.array([w, x, y, z])

    def quaternion_to_euler(self, w, x, y, z):
        # 俯仰角(Pitch)
        pitch = np.arcsin(2 * (w * y - z * x))
        
        # 翻滚角(Roll)
        roll = np.arctan2(2 * (w * x + z * y), 1 - 2 * (x * x + y * y))
        
        # 偏航角(Yaw)
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        
        return np.array([roll, pitch, yaw])


if __name__=='__main__':
    calculator = Calculate_Position()
    # 目标Q的原始旋转四元数和平移向量 w, x, y, z
    q_original = np.array([-0.4050101114235415, -0.6075839806470373, 0.5201112218917696, -0.44304947011371243])

    print("初始旋转欧拉角为：", calculator.quaternion_to_euler(*q_original))

    p_original = np.array([-0.037342273040243304, -0.34348792978243164, -0.0864955364501239])

    # 以目标Q为参考坐标系的平移向量
    d_vector = np.array([0.0855, 0.17, 0.098])

    # 将d_vector从目标坐标系转换到基坐标系
    d_transformed = calculator.transform_vector_by_quaternion(q_original, d_vector)

    # 计算新的平移向量
    p_new = p_original + d_transformed

    # 新的位姿是相同的旋转四元数和新的平移向量
    q_new = q_original
    p_new = p_new

    print("新的旋转四元数:", q_new)
    print("新的平移向量:", p_new)
```

## 六、软件自启动

### 6.1 sh启动程序

* 创建sh自启动文件

```bash
gedit ~/<工作空间>/scripts/auto_upstart.sh  # 创建自启动脚本，一次无法打开时使用ctrl+C重新启动一次即可。
```

* 添加sh脚本数据

```sh
#!/bin/bash

gnome-terminal --tab "driver" -- bash -c "
cd ~/handling_robot_ros1;
source devel/setup.bash;
roslaunch body_handling_demo body_handling_driver.launch;
exec bash"

gnome-terminal --tab "demo" -- bash -c "
cd ~/handling_robot_ros1;
source devel/setup.bash;
sleep 10;
roslaunch body_handling_demo body_handling_action.launch;
exec bash"
```

* 创建sh权限更改文件

由于复制或者刚创建.sh和.py脚本时，并不能保证一定有可执行权限，所以可通过下列脚本修改终端当前目录下所有python和shell脚本的可执行权限，如下：

```bash
gedit ~/<工作空间>/scripts/set_exe_of_scripts.sh  # 创建自启动脚本，一次无法打开时使用ctrl+C重新启动一次即可。
```

* 添加sh脚本数据

```sh
find ./ -type f -name "*.py" -exec chmod +x {} +

find ./ -type f -name "*.sh" -exec chmod +x {} +
```

* 进入到工作空间目录，执行set_exe_of_scripts.sh脚本，从而修改整个工作空间目录下的.py和.sh脚本权限，使得程序方便启动。

```bash
cd ~/<工作空间>  # 进入到工作空间下

sh ./scripts/set_exe_of_scripts.sh  # 启动修改权限的sh脚本文件
```

### 6.2 开机自启配置

* 打开开机自启程序

首先，点击桌面左下角按钮，之后，点击`启动应用程序`软件。

![相机](.\images\auto_program.png)

* 配置开机自启项

软件启动之后，首先点击`添加`按钮，出现`添加应用程序`页面。之后输入自启动服务的名称（图中是`auto_atom`），再点击`浏览`按钮找到前面写过的`auto_upstart.sh`脚本。最后点击`添加`成功创建服务即可。

![开机自启配置](.\images\get_sh.png)

### 6.3 用户自动登录

机器人开机自启动一般需设置用户自动登录：`设置`=>`用户`=>`解锁...`=>`输入密码`=>`认证`=>`认证成功`=>`打开-自动登录(U)`。

![开机自启配置](.\images\login.png)

**注：**打开新终端输入`reboot`即可将设备直接重启，从而验证开机自启是否配置成功。

## 七、视频演示

<video controls width="960" height="544"> 
    <source src="./video/video.mp4">
</video>
## 八、资源下载

* [采购清单](./source/仓储搬运项目物料采购清单.xls)
* [Python环境](./source/requirements.txt)
* [左臂工具坐标系文件](./source/tool_coordinate_system_left.txt)

* [右臂工具坐标系文件](./source/tool_coordinate_system_right.txt)

* [物料框的Aruco码](./source/Aruco_码.docx)

* [woosh底盘资料](./source/woosh_sources.zip)
