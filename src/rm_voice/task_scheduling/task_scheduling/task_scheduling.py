# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist  
from rmctrl_msgs.msg import VoiceControl
import re
import os
from openai import OpenAI

#目录
# 1.初始化发布者和订阅者
# 2.控制底盘
# 3.
# 4.
class TaskScheduling(Node):
    def __init__(self):
        super().__init__('task_scheduling_node')

        # 订阅语音识别
        self.subscription = self.create_subscription(
            String,
            '/recognized_text',
            self.listener_callback,
            10
        )

        # 发布器
        self.publisher_chassis = self.create_publisher(String, '/chassis_topic', 10)
        self.publisher_turtle = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.publisher_feedback = self.create_publisher(String, '/base_control', 10)
        self.publisher_ai = self.create_publisher(String, '/deepseek', 10)
        self.publisher_founder = self.create_publisher(String, '/founder_topic', 10) 
        
    def listener_callback(self, msg):
        # 1. 先提取原始识别文本，不要覆盖入参msg
        text = msg.data.strip()
        self.get_logger().info(f"收到语音：{text}")


        pub_msg = String()
        pub_msg.data = text


        motion_words = ["前进", "向前", "直走", "后退", "向后", "左移", "向左", "右移", "向右"]
        if any(word in text for word in motion_words):
            self.publisher_chassis.publish(pub_msg)
            return  # 匹配底盘指令，直接结束，不执行后面

        founder_words = ["谁创立", "谁创造", "谁发明", "创始人", "谁做的", "作用", "干什么"]
        if any(word in text for word in founder_words):
            self.publisher_founder.publish(pub_msg)

        else:
            self.publisher_ai.publish(pub_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TaskScheduling()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
