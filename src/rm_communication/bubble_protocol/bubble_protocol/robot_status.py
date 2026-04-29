'''
Robot communication status layer,
and this module may be refactored in the future.
This module will publish the robot status data sent by MCU
to onboard to DDS for other subscriber to receice.
机器人通讯状态层，并且这个模块将来可能会被重构。
该模块会发布MCU发送的机器人状态数据,载入 DDS 以供其他订户接收。
'''

from rclpy.node import Node
import rclpy
import math

from std_msgs.msg import Int8
import tf2_ros
import sensor_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
import rmctrl_msgs # type: ignore
import rmctrl_msgs.msg # type: ignore
import numpy as np
from std_msgs.msg import Float32MultiArray
from bubble_protocol.protocol import *
from rmctrl_msgs.msg import ArmStateData

class RobotStatus():
    """Send robot status information.
        发送机器人状态信息。
    Attributes
    ----------
    node: `Node`
        Node of maintain robot status information.
    status: `STATUS`
        Robot current status data.
    """

    def __init__(self, status: dict, node: Node) -> None:
        self.node = node
        self.name = node.name
        self.status = status
        self.realtime_callback = REALTIME_CALLBACK ###定义接受MCU数据的全部回调函数###
        self.status_init()

        # 定时发布非实时状态
        # self.non_realtime_timer = self.node.create_timer(
        #     0.1, self.non_realtime_status)

    def status_init(self) -> None:
        ''' The function defines publishes required for the robot status.
            该函数定义机器人状态所需的发布。
        '''
#下面的回调函数是将接到的mcu的数据通过话题整理后发给别的地方用
        def chassis_callback():
            odom_msg = nav_msgs.msg.Odometry()
            odom_msg.header.stamp = self.node.get_clock().now().to_msg()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_footprint"
            odom_msg.twist.twist.linear.x = float(
                self.status["chassis_ctrl"]["chassis_target_linear_x"][IDX_VAL])
            odom_msg.twist.twist.linear.y = float(
                self.status["chassis_ctrl"]["chassis_target_linear_y"][IDX_VAL])
            odom_msg.twist.twist.linear.z = float(
                self.status["chassis_ctrl"]["chassis_target_linear_z"][IDX_VAL])
            odom_msg.twist.twist.angular.x = float(
                self.status["chassis_ctrl"]["chassis_target_angular_x"][IDX_VAL])
            odom_msg.twist.twist.angular.y = float(
                self.status["chassis_ctrl"]["chassis_target_angular_y"][IDX_VAL])
            odom_msg.twist.twist.angular.z = float(
                self.status["chassis_ctrl"]["chassis_target_angular_z"][IDX_VAL])
            self.odom_pub.publish(odom_msg)
            # self.chassis_pub.publish(chassis_msg)

        def chassis_imu_callback():
            chassis_imu_msg = sensor_msgs.msg.Imu()
            chassis_imu_msg.header.stamp = self.node.get_clock().now().to_msg()
            chassis_imu_msg.header.frame_id = "chassis_imu_link"
            q = quaternion_from_euler(
                math.radians(float(
                    self.status["chassis_imu"]["chassis_imu_eul_rol"][IDX_VAL]) % 360),#绿
                -math.radians(float(
                    self.status["chassis_imu"]["chassis_imu_eul_pit"][IDX_VAL]) % 360),#红
                math.radians(float(
                    self.status["chassis_imu"]["chassis_imu_eul_yaw"][IDX_VAL]) % 360)#蓝
            )
            chassis_imu_msg.orientation.x = q[0]
            chassis_imu_msg.orientation.y = q[1]
            chassis_imu_msg.orientation.z = q[2]
            chassis_imu_msg.orientation.w = q[3]
            chassis_imu_msg.linear_acceleration.x = float(
                self.status["chassis_imu"]["chassis_imu_acc_x"][IDX_VAL])
            chassis_imu_msg.linear_acceleration.y = float(
                self.status["chassis_imu"]["chassis_imu_acc_y"][IDX_VAL])
            chassis_imu_msg.linear_acceleration.z = float(
                self.status["chassis_imu"]["chassis_imu_acc_z"][IDX_VAL])
            chassis_imu_msg.angular_velocity.x = math.radians(float(
                self.status["chassis_imu"]["chassis_imu_angle_x"][IDX_VAL]))
            chassis_imu_msg.angular_velocity.y = math.radians(float(
                self.status["chassis_imu"]["chassis_imu_angle_y"][IDX_VAL]))
            chassis_imu_msg.angular_velocity.z = math.radians(float(
                self.status["chassis_imu"]["chassis_imu_angle_z"][IDX_VAL]))
            self.chassis_imu_pub.publish(chassis_imu_msg)

        def chassis_cmd_from_mcu_callback():
            chassis_cmd_from_mcu_msg = geometry_msgs.msg.Twist()
            chassis_cmd_from_mcu_msg.linear.x = float(
                self.status["chassis_cmd_from_mcu"]["chassis_vx_cmd_from_mcu"][IDX_VAL])
            chassis_cmd_from_mcu_msg.linear.y = float(
                self.status["chassis_cmd_from_mcu"]["chassis_vy_cmd_from_mcu"][IDX_VAL])
            chassis_cmd_from_mcu_msg.angular.z = float(
                self.status["chassis_cmd_from_mcu"]["chassis_vw_cmd_from_mcu"][IDX_VAL])
            self.chassis_cmd_from_mcu.publish(chassis_cmd_from_mcu_msg)

        def chassis_odom_callback():
            # 发布tf2的odom变换
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = self.node.get_clock().now().to_msg()
            t.header.frame_id = "odom"
            t.child_frame_id = "base_footprint"
            t.transform.translation.x = float(
                self.status["chassis_odom"]["odom_position_x"][IDX_VAL])
            t.transform.translation.y = float(
                self.status["chassis_odom"]["odom_position_y"][IDX_VAL])
            t.transform.translation.z = 0.0
            q = quaternion_from_euler(0, 0, float(
                self.status["chassis_odom"]["odom_angle"][IDX_VAL]))
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.odom_br.sendTransform(t)

            chassis_odom_msg = nav_msgs.msg.Odometry()
            chassis_odom_msg.header.stamp = self.node.get_clock().now().to_msg()
            chassis_odom_msg.header.frame_id = "odom"
            chassis_odom_msg.child_frame_id = "base_footprint"
            chassis_odom_msg.pose.pose.position.x = float(
                self.status["chassis_odom"]["odom_position_x"][IDX_VAL])
            chassis_odom_msg.pose.pose.position.y = float(
                self.status["chassis_odom"]["odom_position_y"][IDX_VAL])

            chassis_odom_msg.pose.pose.orientation.x = q[0]
            chassis_odom_msg.pose.pose.orientation.y = q[1]
            chassis_odom_msg.pose.pose.orientation.z = q[2]
            chassis_odom_msg.pose.pose.orientation.w = q[3]

            chassis_odom_msg.twist.twist.linear.x = float(
                self.status["chassis_odom"]["odom_velocity_x"][IDX_VAL])
            chassis_odom_msg.twist.twist.linear.y = float(
                self.status["chassis_odom"]["odom_velocity_y"][IDX_VAL])
            chassis_odom_msg.twist.twist.angular.z = float(
                self.status["chassis_odom"]["odom_angular_velocity"][IDX_VAL])
            self.odom_pub.publish(chassis_odom_msg)

            

        def quaternion_from_euler(ai, aj, ak):
            ai /= 2.0
            aj /= 2.0
            ak /= 2.0
            ci = math.cos(ai)
            si = math.sin(ai)
            cj = math.cos(aj)
            sj = math.sin(aj)
            ck = math.cos(ak)
            sk = math.sin(ak)
            cc = ci * ck
            cs = ci * sk
            sc = si * ck
            ss = si * sk

            q = np.empty((4,))
            q[0] = cj * sc - sj * cs
            q[1] = cj * ss + sj * cc
            q[2] = cj * cs - sj * sc
            q[3] = cj * cc + sj * ss

            return q

        def joints_status_from_mcu_callback():
            joint_state_sub_from_mcu_msg = ArmStateData()
            joint_state_sub_from_mcu_msg.joint1_position = float(self.status["joint_state_sub_from_mcu"]["joint1_position"][IDX_VAL])
            joint_state_sub_from_mcu_msg.joint2_position = float(self.status["joint_state_sub_from_mcu"]["joint2_position"][IDX_VAL])
            joint_state_sub_from_mcu_msg.joint3_position = float(self.status["joint_state_sub_from_mcu"]["joint3_position"][IDX_VAL])
            joint_state_sub_from_mcu_msg.joint4_position = float(self.status["joint_state_sub_from_mcu"]["joint4_position"][IDX_VAL])
            joint_state_sub_from_mcu_msg.joint5_position = float(self.status["joint_state_sub_from_mcu"]["joint5_position"][IDX_VAL])
            joint_state_sub_from_mcu_msg.joint6_position = float(self.status["joint_state_sub_from_mcu"]["joint6_position"][IDX_VAL])
            
            joint_state_sub_from_mcu_msg.joint1_velocity = float(self.status["joint_state_sub_from_mcu"]["joint1_velocity"][IDX_VAL])
            joint_state_sub_from_mcu_msg.joint2_velocity = float(self.status["joint_state_sub_from_mcu"]["joint2_velocity"][IDX_VAL])
            joint_state_sub_from_mcu_msg.joint3_velocity = float(self.status["joint_state_sub_from_mcu"]["joint3_velocity"][IDX_VAL])
            joint_state_sub_from_mcu_msg.joint4_velocity = float(self.status["joint_state_sub_from_mcu"]["joint4_velocity"][IDX_VAL])
            joint_state_sub_from_mcu_msg.joint5_velocity = float(self.status["joint_state_sub_from_mcu"]["joint5_velocity"][IDX_VAL])
            joint_state_sub_from_mcu_msg.joint6_velocity = float(self.status["joint_state_sub_from_mcu"]["joint6_velocity"][IDX_VAL])
            joint_state_sub_from_mcu_msg.gripper_state = int(self.status["joint_state_sub_from_mcu"]["gripper_state"][IDX_VAL]) #通过这个状态控制自动抓取功能的开关
            joint_state_sub_from_mcu_msg.auto_state = int(self.status["joint_state_sub_from_mcu"]["auto_state"][IDX_VAL])
            joint_state_sub_from_mcu_msg.arm_ctrl_mode = int(self.status["joint_state_sub_from_mcu"]["arm_ctrl_mode"][IDX_VAL])
            self.joint_state_sub_from_mcu_pub.publish(joint_state_sub_from_mcu_msg)


        # real-time publisher api
        if self.name == "engineer":###定义NUC现在是那个机器人在用###
            ###在此处添加需要发布出去给大家的MCU话题数据###
            self.joint_state_sub_from_mcu_pub = self.node.create_publisher(
                ArmStateData,'/joint_state_sub_from_mcu', 10)

            ###在此处添加机器人需要从MCU接收的话题数据###
            self.realtime_callback["joint_state_sub_from_mcu"] = joints_status_from_mcu_callback