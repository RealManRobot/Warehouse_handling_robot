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
