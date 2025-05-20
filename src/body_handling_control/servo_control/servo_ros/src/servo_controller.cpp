#include <iostream>
#include <iomanip>

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include <servo_ros/ServoMove.h>
#include <servo_ros/ServoAngle.h>


serial::Serial ros_ser;
#define sBUFFERSIZE 10 // send buffer size 串口发送缓存长度
unsigned char s_buffer[sBUFFERSIZE]; // 发送缓存

// 舵机转动控制回调函数
void callback_servoMove(const servo_ros::ServoMove& msg){
    memset(s_buffer, 0, sizeof(s_buffer));
    s_buffer[0] = 0x55;
    s_buffer[1] = 0x55;
    s_buffer[2] = 0x08;
    s_buffer[3] = 0x03;
    s_buffer[4] = 0x01;
    s_buffer[5] = 0xe8;
    s_buffer[6] = 0x03;
    s_buffer[7] = msg.servo_id;  // 舵机ID
    s_buffer[8] = msg.angle; // 角度位置低八位
    s_buffer[9] = msg.angle >> 8; // 角度位置高八位
    ros_ser.write(s_buffer, sBUFFERSIZE);
    ROS_INFO_STREAM("Control Servo Move");
}


// 读取两个舵机角度位置值函数
void servoGetAngle(){
    memset(s_buffer, 0, sizeof(s_buffer));
    s_buffer[0] = 0x55;
    s_buffer[1] = 0x55;
    s_buffer[2] = 0x05;
    s_buffer[3] = 0x15;
    s_buffer[4] = 0x02;
    s_buffer[5] = 0x01;  // 舵机ID
    s_buffer[6] = 0x02;  // 舵机ID
    ros_ser.write(s_buffer, 7);
    //ROS_INFO_STREAM("Get Servo Angle");
}




int main(int argc, char** argv) {
    
    ros::init(argc, argv, "my_serial_node");
    ros::NodeHandle n;
    
    // 订阅控制舵机转动主题
    ros::Subscriber sub_servo_move = n.subscribe("/servo_control/move", 1000, callback_servoMove);
    
    // 发布主题:舵机状态
    ros::Publisher pub_servo_state = n.advertise<servo_ros::ServoAngle>("/servo_state", 1000);

    try {
        ros_ser.setPort("/dev/ttyUSB0");
        ros_ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ros_ser.setTimeout(to);
        ros_ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if (ros_ser.isOpen()) {
        ROS_INFO_STREAM("Serial Port opened");
    } else {
        return -1;
    }

    ros::Rate loop_rate(10);
    
    while (ros::ok()) {
        ros::spinOnce();
        servoGetAngle();

        // 获取缓冲区内的字节数
        size_t n = ros_ser.available();
        if (n != 0) {
            uint8_t buffer[1024];
            // 读出数据
            n = ros_ser.read(buffer, n);
            // ROS_INFO_STREAM("Reading from serial port");

            if (buffer[0] == 0x55 && buffer[1] == 0x55) {
                int servo_id_1 = buffer[n - 6];
                int angle_1 = (buffer[n - 4] << 8) | buffer[n - 5];
                int servo_id_2 = buffer[n - 3];
                int angle_2 = (buffer[n - 1] << 8) | buffer[n - 2];
                // std::cout << "Servo ID 1 : " << servo_id_1 << ", Angle1: " << angle_1 << "        Servo ID2: " << servo_id_2 << ", Angle2: " << angle_2 << std::endl;
                servo_ros::ServoAngle msg;
                msg.servo_id_1 = servo_id_1;
                msg.angle_1 = angle_1;
                msg.servo_id_2 = servo_id_2;
                msg.angle_2 = angle_2;
                pub_servo_state.publish(msg);
                
            }
        }

        loop_rate.sleep();
    }
    ros_ser.close();
}

