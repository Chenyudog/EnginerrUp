cmds=(
      "ros2 launch astra_camera astra.launch.xml" #启动深度相机
      "ros2 run energy_lattice_detect energy_rec" #启动能量单元识别节点
     )
#节点名称： ros2 topic echo energy_lattice_position
for cmd in "${cmds[@]}";
do
     echo Current CMD : "$cmd"
     gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
     sleep 1.0
done
