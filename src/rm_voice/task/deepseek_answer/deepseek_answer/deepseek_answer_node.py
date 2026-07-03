# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
# 补充导入
from openai import OpenAI

class DeepSeekAnswer(Node):
    def __init__(self):
        super().__init__('deepseek_answer_node')

        # 订阅语音识别话题 /deepseek
        self.subscription = self.create_subscription(
            String,
            '/deepseek',
            self.listener_callback,
            10
        )

        # AI回答发布器
        self.publisher_feedback = self.create_publisher(String, '/voice_feedback', 10)

        # 仅初始化一次DeepSeek客户端
        self.client = None
        api_key = os.environ.get('DEEPSEEK_API_KEY')
        if api_key:
            self.client = OpenAI(
                api_key=api_key,
                base_url="https://api.deepseek.com"
            )
            self.get_logger().info("DeepSeek 客户端初始化成功")
        else:
            self.get_logger().error("未检测到环境变量 DEEPSEEK_API_KEY，AI功能不可用")

    def listener_callback(self, msg):
        # 提取用户文本
        text = msg.data.strip()
        self.get_logger().info(f"收到用户提问：{text}")


        # 调用DeepSeek，移除不兼容参数
        response = self.client.chat.completions.create(
            model="deepseek-v4-pro",
            messages=[
                {
                    "role": "system",
                    "content": "今天是2026年7月2日星期四，你是一个生活管家，什么问题都能解答，用极简短句我的问题"
                },
                {"role": "user", "content": text}
            ],
            stream=False
        )
        ai_reply = response.choices[0].message.content.strip()
        self.get_logger().info(f"AI回复：{ai_reply}")

        # 封装ROS消息并发布
        feedback_msg = String()
        feedback_msg.data = ai_reply
        self.publisher_feedback.publish(feedback_msg)



def main(args=None):
    rclpy.init(args=args)
    node = DeepSeekAnswer()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()