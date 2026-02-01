cmds=(
      "ros2 launch bubble_protocol engineer_up_serial_launch.py" #启动与机械臂的通信
      "ros2 launch engineer_bringup engineer_moveit2.launch.xml" #启动moveit2
      "ros2 topic echo /joint_state_sub_from_mcu" #读取下位机回馈的数据
      "ros2 topic echo /joint_cmd_from_moveit2" #读取moveit2发送的数据
     )
     
for cmd in "${cmds[@]}";
do
     echo Current CMD : "$cmd"
     gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
     sleep 1.0
done
