# Engineering_Up 项目配置指南新机要下载

## 新机下载
sudo apt install -y ros-humble-pcl-ros 
sudo apt install ros-humble-xacro
sudo apt install ros-humble-joint-state-publisher
pip3 install pyserial   下载串口包
-----------------------------------------------------------------------

## [初始配置mid360网址]
(https://blog.csdn.net/hong_j_0826/article/details/142236061?ops_request_misc=elastic_search_misc&request_id=054fb5058c3662aaeb9b98fe1ec9e704&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~ElasticSearch~search_v2-1-142236061-null-null.142^v102^pc_search_result_base3&utm_term=ros2%20humble%20%E5%A6%82%E4%BD%95%E5%88%9D%E6%AD%A5%E4%BD%BF%E7%94%A8Livox-mid-%E6%BF%80%E5%85%89%E9%9B%B7%E8%BE%BE&spm=1018.2226.3001.4187)


## 如何配置代码于自己电脑
1、git clone https://github.com/poppywork/EnginerrUp.git
2、cd src/rm_driver
3、git clone https://github.com/Livox-SDK/Livox-SDK2.git
4、cd ./Livox-SDK2/
5、mkdir build 
6、cd build
7、cmake .. && make -j8
8、sudo make install
9、cd ..   cd ..
10、cd livox_ros_driver2
11、source /opt/ros/humble/setup.sh
12、./build.sh humble  
13、cd 自己工作空间
14、colcon build (如果显示黄色,再构建一次)



## 连接雷达权限需求
sudo ifconfig查看自己电脑网口名称(我的是enp89s0)
//给雷达临时权限指令
sudo ifconfig enp89s0 192.168.2.50  

//给雷达永久权限指令
1、cd  /ect/netplan/
2、sudo gedit 01-network-manager-all.yaml  (没有就创建一个)
3、输入:ethernets: #注意缩进
    enp89s0:  
      addresses: [192.168.2.50/24]  
4、sudo netplan apply //执行应用命令


## 连接开发板
lsusb
查看开发板序列号类似:Bus 003 Device 004: ID 0483:5740 STMicroelectronics Virtual COM Port  # Vendor ID (ID 后的前 4 位) 和 Product ID (后 4 位)，
sudo gedit /etc/udev/rules.d/99-stm32.rules 新建stm32开发板规则文件
输入: SUBSYSTEM=="tty", ATTRS{idVendor}=="你的VendorID", ATTRS{idProduct}=="你的ProductID", SYMLINK+="STM32H7", MODE="0666"
sudo udevadm control --reload-rules
sudo udevadm trigger  # 重新触发设备检测
ls -l /dev/STM32H7    #检查是否生成了 /dev/STM32H7

## 构建地图+保存地图
1、启动 mapping.sh 开始建图
2、cd maps目录
3、ros2 run nav2_map_server map_saver_cli -f room202-5 保存地图

### 建图导航小tips
1、/home/zrk/EnginerrUp/src/rm_navigation/navi/params/nav2_params.yaml 在这个文件修改导航的参数
官方文档在https://fishros.org/doc/nav2/plugins/index.html#planners
2、


## 配置Moveit2环境指令:
sudo apt install ros-humble-moveit（报错了就修复一下源，报错AI可以解答你）
sudo apt install ros-humble-moveit-setup-assistant
sudo apt install ros-humble-moveit-*

网址:https://blog.csdn.net/joyopirate/article/details/129424607?ops_request_misc=elastic_search_misc&request_id=5c41293e69b7297bf39eefe8573b74ef&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_click~default-1-129424607-null-null.142^v102^pc_search_result_base8&utm_term=%E5%9C%A8ros2%E4%B8%AD%EF%BC%8C%E9%80%9A%E8%BF%87Moveit2&spm=1018.2226.3001.4187

## 重新修改moveit2模型
1、正常导出urdf文件夹
2、修改joint_limits.yaml文件里的报错，将整数改为小数(还有initial_position的)
3、在moveit_controllers.yaml的控制器后面加上
    action_ns: follow_joint_trajectory
    default: true
4、将新出的config的文件覆盖掉原来的engineer_bringup里的yaml文件(ros2_controller文件)
5、在urdf文件更新ros2_control标签的内容，把${initial_positions['joint_gripper_right']}这些换成想要的数字，比如0.0
6、打开engineer_bingup  launch文件启动机器人
7、启动控制器
8、向控制器发布话题即可

##如何继续部署到工程车上:
1、修改engineer_moveit2.launch.xml上的包名，文件名等



##如何配置深度相机环境
#1.配必要文件
sudo apt install libgflags-dev  ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager\
ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev
sudo apt install -y nlohmann-json3-dev
以下代码在任意目录下皆可
git clone https://github.com/libuvc/libuvc.git
cd libuvc
mkdir build && cd build
cmake .. && make -j4
sudo make install
sudo ldconfig # Refreshing the link library
#2.创建工作空间
mkdir ~/工作空间/src
提取文件    并放在src目录下(替换掉源代码的ros2_astra_camera文件夹) 
链接: https://pan.baidu.com/s/1Z1ljB5KPBhKWVNzE-obqsw?pwd=i485 提取码: i485 

cd ~/工作空间/src/ros2_astra_camera/astra_camera/scripts
sudo bash install.sh
sudo udevadm control --reload-rules && sudo udevadm trigger

编译：
 cd ~/ros2_ws
source /opt/ros/humble/setup.bash 
colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release
驱动：(根据自己相机打开对应的launch 文件）
source /opt/ros/humble/setup.bash 
source ./install/setup.bash 
ros2 launch astra_camera astra.launch.xml

