#include <ros/ros.h>
#include <iostream>
#include <unistd.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/Pose.h>
#include <dual_arm_msgs/ChangeWorkFrame_Name.h>

using namespace std;


// 机械臂状态类
class Listen_Arm_State
{
public:
    // 话题发布者声明
    string left_top_name, right_top_name, left_camera_name, right_camera_name, midl_camera_name, midr_camera_name;
    // 基坐标系名称
    string left_base_name, right_base_name;

    // 构造函数
    Listen_Arm_State(ros::NodeHandle &nh);
    // 析构函数
    ~Listen_Arm_State();

private:
    // 更改工作坐标系
    ros::Publisher l_change_work_frame;
    ros::Publisher r_change_work_frame;

    // 话题订阅者声明
    ros::Subscriber l_state_sub;
    ros::Subscriber r_state_sub;

    string open_base;


    // 机械臂状态监听回调函数声明
    void Right_Arm_Current_State_Callback(const geometry_msgs::Pose::ConstPtr &msg);
    void Left_Arm_Current_State_Callback(const geometry_msgs::Pose::ConstPtr &msg);
};

Listen_Arm_State::Listen_Arm_State(ros::NodeHandle &nh)
{
    // 获取坐标系数据
    nh.param<string>("right_base_name", right_base_name, "base_right");
    nh.param<string>("left_base_name", left_base_name, "base_left");
    nh.param<string>("right_top_name", right_top_name, "top_right");
    nh.param<string>("left_top_name", left_top_name, "top_left");
    nh.param<string>("left_camera_name", left_camera_name, "camera_left");
    nh.param<string>("right_camera_name", right_camera_name, "camera_right");
    nh.param<string>("midl_camera_name", midl_camera_name, "camera_middle_left");
    nh.param<string>("midr_camera_name", midr_camera_name, "camera_middle_right");
    nh.param<string>("open_base", open_base, "World");
    
    l_change_work_frame = nh.advertise<dual_arm_msgs::ChangeWorkFrame_Name>("/l_arm/rm_driver/ChangeWorkFrame_Cmd", 10);
    r_change_work_frame = nh.advertise<dual_arm_msgs::ChangeWorkFrame_Name>("/r_arm/rm_driver/ChangeWorkFrame_Cmd", 10);

    // 状态监听
    r_state_sub = nh.subscribe<geometry_msgs::Pose>("/r_arm/rm_driver/Pose_State", 1, 
                              &Listen_Arm_State::Right_Arm_Current_State_Callback, this);
    l_state_sub = nh.subscribe<geometry_msgs::Pose>("/l_arm/rm_driver/Pose_State", 1,
                              &Listen_Arm_State::Left_Arm_Current_State_Callback, this);

    // 循环检查是否有订阅者
    while (ros::ok()) {
        ros::spinOnce(); // 处理回调函数
        if (l_change_work_frame.getNumSubscribers() > 0 && r_change_work_frame.getNumSubscribers() > 0) {
            // 更改工作坐标系
            dual_arm_msgs::ChangeWorkFrame_Name work_frame;
            work_frame.WorkFrame_name = this->open_base;
            // 发布工作坐标系
            l_change_work_frame.publish(work_frame);
            r_change_work_frame.publish(work_frame);
            break;
        } else {
            ROS_INFO("Wait for the robotic arm interface to open ..."); // 话题没有订阅者
        }
        ros::Duration(1.0).sleep(); // 等待1秒
    }
}

Listen_Arm_State::~Listen_Arm_State()   {}

void Listen_Arm_State::Right_Arm_Current_State_Callback(const geometry_msgs::Pose::ConstPtr &msg)
{
    // 创建tf的广播器
    static tf::TransformBroadcaster r_base_top;
    // 初始化tf数据
    tf::Transform r_transform;
    r_transform.setOrigin(tf::Vector3(msg->position.x, msg->position.y, msg->position.z));
    tf::Quaternion r_q;
    r_q.setW(msg->orientation.w);
    r_q.setX(msg->orientation.x);
    r_q.setY(msg->orientation.y);
    r_q.setZ(msg->orientation.z);
    r_transform.setRotation(r_q);

    // 广播flange与base之间的tf数据
    r_base_top.sendTransform(tf::StampedTransform(r_transform, ros::Time::now(), right_base_name, right_top_name));
}

void Listen_Arm_State::Left_Arm_Current_State_Callback(const geometry_msgs::Pose::ConstPtr &msg)
{
    // 创建tf的广播器
    static tf::TransformBroadcaster l_base_top;
    // 初始化tf数据
    tf::Transform l_transform;
    l_transform.setOrigin( tf::Vector3(msg->position.x,msg->position.y,msg->position.z));
    tf::Quaternion l_q;
    l_q.setW(msg->orientation.w);
    l_q.setX(msg->orientation.x);
    l_q.setY(msg->orientation.y);
    l_q.setZ(msg->orientation.z);
    
    l_transform.setRotation(l_q);

    // 广播flange与base之间的tf数据
    l_base_top.sendTransform(tf::StampedTransform(l_transform, ros::Time::now(), left_base_name, left_top_name));
}


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
        camera_transform_l.setOrigin(tf::Vector3(0.103744, -0.040517, 0.009167));
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
        camera_transform_r.setOrigin(tf::Vector3(0.103744, -0.043517, 0.009167));
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

        camera_transform_midl.setOrigin(tf::Vector3(-0.146211, -0.102476, 0.110149));
        tf::Quaternion quater_midl;
        quater_midl.setW(0.343942);
        quater_midl.setX(0.182327);
        quater_midl.setY(0.794347);
        quater_midl.setZ(-0.466342);
        camera_transform_midl.setRotation(quater_midl);

        // 广播flange与base之间的tf数据
        m_top_camera.sendTransform(tf::StampedTransform(camera_transform_midl, ros::Time::now(),
                                   arm_state_publisher.left_base_name, arm_state_publisher.midl_camera_name));

        // 初始化头部tf数据, 右头部相机标定
        tf::Transform camera_transform_midr;

        camera_transform_midr.setOrigin(tf::Vector3(0.204049, -0.100638, 0.065227));
        tf::Quaternion quater_midr;
        quater_midr.setW(0.320181);
        quater_midr.setX(0.202702);
        quater_midr.setY(-0.813887);
        quater_midr.setZ(0.440436);
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
