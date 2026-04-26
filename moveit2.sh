#!/bin/bash
ROS2_BASE="/opt/ros/humble"          # ROS2 安装路径
WORKSPACE_PATH="/home/zrk/EnginerrUp"   #  ROS2 工作空间根目录

sleep 5.0


cmds=(
    "ros2 launch bubble_protocol engineer_up_serial_launch.py"
    "ros2 launch rm_bringup engineer_moveit2.launch.xml"
    "ros2 topic echo /joint_state_sub_from_mcu"
    "ros2 topic echo /joint_cmd_from_moveit2"
)

for cmd in "${cmds[@]}"; do
    echo "Current CMD : $cmd"
    gnome-terminal -- bash -c "
        source $ROS2_BASE/setup.bash;
        cd $WORKSPACE_PATH;
        source install/setup.bash;
        $cmd;
        exec bash
    "
    sleep 2.0
done