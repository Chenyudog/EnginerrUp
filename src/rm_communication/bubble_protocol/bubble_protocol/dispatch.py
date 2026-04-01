'''
Robot communication dispatch layer, 
this module is the bridge between DDS and serial port hardware. 
Through the module, 
a BCP frame can be initialized to receive the data transmitted by DDS 
and send the data to MCU. 
At the same time, the data received by the onboard computer will also be 
transmitted to the DDS through this module 
to be received by other subscribers.
机器人通信调度层,该模块是DDS(内存)与串口硬件之间的桥梁。
通过模块,可以初始化一个BCP帧来接收DDS传输的数据并将数据发送给MCU。
同时,车载电脑接收到的数据也会通过该模块传输到DDS,由其他订户接收。
'''
import math
import time

from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovariance, Twist

from bubble_protocol.hardware import RobotSerial
from bubble_protocol.robot_status import RobotStatus
from rmctrl_msgs.msg import  Imu  
from rmctrl_msgs.msg import ArmCtrlData


class RobotAPI(Node):
    """Generate a new BCP core.

    Attributes
    ----------
    robot_status: `RobotStatus`
        The robot current status.
    robot_serial: `Serial`
        Onboard serial port object.

    Examples
    --------
    >>> from bubble_protocol.dispatch import RobotAPI
    >>> core = RobotAPI()
    """

    def __init__(self) -> None:  #默认启动机械臂工程车
        """
        Generate a new BCP core.
        初始化一个新的 BCP core类
        """
        super().__init__("BCP_Core")

        self.declare_parameter('robot_type', 'None')#这两个参数都在launch文件定义好了
        self.declare_parameter('serial_port', 'None')
        self.name = self.get_parameter(
            'robot_type').get_parameter_value().string_value
        self.serial_port = self.get_parameter(
            'serial_port').get_parameter_value().string_value
        if self.name != 'None':
            self.get_logger().info(f'Robot {self.name} has been initialized...')
        else:
            self.get_logger().error(
                f'No robot has been initialized, please check the parameter Settings!')
            self.get_logger().error(
                f'Robot type:{self.name}')
            raise ValueError(
                'No robot has been initialized, please check the parameter Settings!')

        serial_opened = False
        while not serial_opened:
            try:
                #在这里进行机器人初始化和串口初始化
                self.robot_serial = RobotSerial(self.name, port=self.serial_port)
                self.get_logger().info(f'Serial Port {self.serial_port} has been initialized...')
                serial_opened = True
            except Exception as e:
                self.get_logger().error(
                    f'Open serial port error, try to reopen port:{self.serial_port}, info: {e}')
                time.sleep(3)

        self.robot_status = RobotStatus(self.robot_serial.status, self)###注册与MCU关联的话题和回调函数层
        self.robot_serial.realtime_pub = self.robot_status.realtime_callback
        self.robot_serial.serial_done = True

        self.api_init()###注册与NUC关联的订阅话题和回调函数层

        ###发送流程：话题回调，将数据放在txbuffer里面，定时回调process函数发送出去
        self.uart_timer = self.create_timer(0.01, self.robot_serial.process)
        ###接收流程：定时回调process函数接受串口缓冲区数据，并解析出数据包->
        ###定时回调函数rx_function将数据包分类调用写好的接收回调函数
        self.uartrx_timer = self.create_timer(
            0.01, self.robot_serial.rx_function)

        ### rx_function读取数据解析出符合格式数据包，process将解析的数据包具体分配到protocol定义好的数据包里面
        ### 和将发送缓冲区里的数据包发送出去

    def api_init(self) -> None:
        #在此添加需要从上位机别的地方发来的话题,通过回调函数发出去
        # 前提是robot_status文件里面写了对于的话题名称和对于消息类型
        if self.name == "engineer":
            self.chassis_sub = self.create_subscription(
                Twist, '/cmd_vel', self.ex_chassis_callback, 10)
            self.odom_sub = self.create_subscription(
                Twist, '/odom', self.ex_odom_callback, 10)
            self.joint_state_sub = self.create_subscription(
                ArmCtrlData, '/joint_cmd_from_moveit2', self.ex_joint_state_sub_from_moveit2_callback, 10)

    def gimbal_callback(self, msg: Imu) -> None:
        mode = 1
        self.robot_serial.send_data(
            "gimbal",
            [mode, math.degrees(msg.yaw), math.degrees(msg.pitch), 0, 0, 0, 0])
        print(f"send data yaw:{msg.yaw},pitch{msg.pitch},pitch{msg.roll}")

    def ex_chassis_callback(self, msg: Twist) -> None:
        print("接收到上位机对下位机底盘的控制命令，现在发出")
        self.robot_serial.send_data("chassis_ctrl", [msg.linear.x, msg.linear.y, msg.linear.z,
                                                     msg.angular.x, msg.angular.y, -msg.angular.z])

    def ex_odom_callback(self, msg: PoseWithCovariance):
        print("接收到里程计,现在通过串口发送出去")
        odom_list = []
        odom_list.append(msg.pose.position.x)
        odom_list.append(msg.pose.position.y)
        odom_list.append(msg.pose.position.z)
        odom_list.append(msg.pose.orientation.x)
        odom_list.append(msg.pose.orientation.y)
        odom_list.append(msg.pose.orientation.z)
        odom_list.append(msg.pose.orientation.w)
        self.robot_serial.send_data("odom", odom_list)

    def ex_joint_state_sub_from_moveit2_callback(self, msg: ArmCtrlData):
        print("接收到上位机对下位机机械臂的控制命令,现在通过串口发送出去")
        joint_cmd_sub_from_moveit2_list = []
        joint_cmd_sub_from_moveit2_list.append(msg.joint1_position)
        joint_cmd_sub_from_moveit2_list.append(msg.joint2_position)
        joint_cmd_sub_from_moveit2_list.append(msg.joint3_position)
        joint_cmd_sub_from_moveit2_list.append(msg.joint4_position)
        joint_cmd_sub_from_moveit2_list.append(msg.joint5_position)
        joint_cmd_sub_from_moveit2_list.append(msg.joint6_position)
        joint_cmd_sub_from_moveit2_list.append(msg.gripper_ctrl)
        joint_cmd_sub_from_moveit2_list.append(msg.auto_state)


        self.robot_serial.send_data("joint_cmd_from_moveit2", joint_cmd_sub_from_moveit2_list)