cmds=(
      "ros2 launch livox_ros_driver2 msg_MID360_cloud_launch.py" #连接雷达
      "ros2 launch engineer_bringup model.launch.xml" #发布静态tf
      "ros2 launch linefit_ground_segmentation_ros segmentation.launch.py" #对 3D 点云进行地面分割
      "ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py" #压缩3D雷达为2D
      "ros2 launch point_lio mapping_mid360.launch.py" #启动point_lio，发布里程计
      "ros2 launch bubble_protocol engineer_up_serial_launch.py" #连接下位机
      "ros2 launch navi navigation_launch.py" #启动导航
     )
     
for cmd in "${cmds[@]}";
do
     echo Current CMD : "$cmd"
     gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
     sleep 0.2
done
