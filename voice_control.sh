cmds=(
	"ros2 run asr asr_node" #语音识别
	"ros2 launch online_asr voice_control_with_feedback.launch.py" #语音控制底盘
	"ros2 launch bubble_protocol engineer_up_serial_launch.py serial_port:=/dev/ttyACM0"  # 打开上下位机通信串口
	"ros2 run engineer_info engineer_info_node" #工程车个性化表达
	"ros2 launch voice_picture_detect voice_picture_detect.launch.xml"  #语音描述图像
	"ros2 run tts tts_node" #语音合成
     )
     
for cmd in "${cmds[@]}";
do
     echo Current CMD : "$cmd"
     gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
     sleep 0.2
done
