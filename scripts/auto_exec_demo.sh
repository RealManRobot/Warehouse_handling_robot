#!/bin/sh

# 预设检测时长
ALL_TIME=120 # s
# 最小时常，等待机械臂连接
MIN_TIME=20 # s

# 主控机IP（运行roscore的主机）
MASTER_IP="169.254.128.2" # 假设这是运行的主机ip
# 从机IP（根据实际情况设置或自动获取）
CURRENT_IP="169.254.128.20"

# ip组
IP_LIST="169.254.128.18 169.254.128.19 169.254.128.2 169.254.128.20"
echo "开始检测IP连通性..."

MAX_ATTEMPTS=30
IP_INTERVAL=1
all_reachable=false
attempt=0

while [ "$all_reachable" = false ] && [ $attempt -lt $MAX_ATTEMPTS ]; do
    all_reachable=true
    attempt=$((attempt+1))
    clear
    
    echo "尝试 $((attempt))/$MAX_ATTEMPTS:"
    echo "------------------------"
    
    for ip in $IP_LIST; do
        if ping -c 1 -W 1 "$ip" >/dev/null 2>&1; then
            echo "[✓] $ip 可达"
        else
            echo "[×] $ip 不可达"
            all_reachable=false
        fi
    done
    
    [ "$all_reachable" = true ] && break
    
    echo "------------------------"
    echo "等待 $IP_INTERVAL 秒后重试..."
    sleep $IP_INTERVAL
done

[ "$all_reachable" = false ] && exit 1

# 等待机械臂初始化
sleep $(( (MIN_TIME - attempt * IP_INTERVAL) > 0 ? (MIN_TIME - attempt * IP_INTERVAL) : 0 ))

if [ "$all_reachable" = true ]; then
    echo "All devices found! Proceeding..."
    
    # 设置ROS环境变量（关键修改部分）
    export ROS_MASTER_URI="http://${MASTER_IP}:11311"
    export ROS_IP="${CURRENT_IP}"
    
    # 启动驱动节点
    gnome-terminal --tab "driver" -- bash -c "
        source ~/.bashrc;
        export DISABLE_ROS1_EOL_WARNINGS=1;
        export ROS_MASTER_URI=http://${MASTER_IP}:11311;
        export ROS_IP=${CURRENT_IP};
        sleep 1;
        cd ~/handling_robot_ros1;
        source devel/setup.bash;
        roslaunch body_handling_demo body_handling_driver.launch;
        exec bash;
    "
    
    sleep 25;

    # 总体demo启动文件
    gnome-terminal --tab "demo" -- bash -c "
        source ~/.bashrc;
        export ROS_MASTER_URI=http://${MASTER_IP}:11311;
        export ROS_IP=${CURRENT_IP};
        sleep 1;
        cd ~/handling_robot_ros1;
        source devel/setup.bash;
        roslaunch body_handling_demo body_handling_action.launch;
        exec bash;
    "
else
    echo "Error: the following ports not found after $MAX_ATTEMPTS attempts:"
    exit 1
fi
