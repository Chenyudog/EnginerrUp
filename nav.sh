cmds=(
     #  "ros2 launch navi slam_launch.py" #启动建图
      "ros2 launch navi navigation_launch.py" #启动导航
     #  "ros2 topic echo /tf"
     #  "ros2 topic echo /cmd_vel"
     )
     
for cmd in "${cmds[@]}";
do
     echo Current CMD : "$cmd"
     gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
     sleep 0.4
done
