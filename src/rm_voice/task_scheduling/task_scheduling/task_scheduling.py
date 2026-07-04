# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist  

class TaskScheduling(Node):
    # 指令关键词常量，统一管理
    MOTION_KEYWORDS = ["前进", "向前", "直走", "后退", "向后", "左移", "向左", "右移", "向右"]
    FOUNDER_KEYWORDS = ["谁创立", "谁创造", "谁发明", "创始人", "谁做的", "作用", "干什么"]
    CAPTURE_KEYWORDS = ["看到了什么", "显示什么"]

    def __init__(self):
        super().__init__('task_scheduling_node')

        # 订阅语音识别文本
        self.subscription = self.create_subscription(
            String,
            '/recognized_text',
            self.listener_callback,
            10
        )

        # 各功能发布器
        self.pub_chassis = self.create_publisher(String, '/chassis_topic', 10)
        self.pub_turtle = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pub_ai = self.create_publisher(String, '/deepseek', 10)
        self.pub_founder = self.create_publisher(String, '/founder_topic', 10)
        self.pub_capture = self.create_publisher(String, '/picture_capture', 10)

    def listener_callback(self, msg):
        text = msg.data.strip()
        self.get_logger().info(f"收到语音识别文本：{text}")

        # 过滤空消息
        if not text:
            self.get_logger().warn("收到空语音文本，忽略")
            return

        pub_msg = String()
        pub_msg.data = text

        # 互斥分支：一条语音只匹配一类任务
        if any(word in text for word in self.MOTION_KEYWORDS):
            self.pub_chassis.publish(pub_msg)

        elif any(word in text for word in self.FOUNDER_KEYWORDS):
            self.pub_founder.publish(pub_msg)

        elif any(word in text for word in self.CAPTURE_KEYWORDS):
            self.pub_capture.publish(pub_msg)

        # 其他全部交给大模型AI处理
        else:
            self.pub_ai.publish(pub_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TaskScheduling()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()