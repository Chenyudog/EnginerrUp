# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class FounderAnswerNode(Node):
    def __init__(self):
        super().__init__("founder_answer_node")

        # 订阅语音识别结果
        self.sub = self.create_subscription(
            String,
            "/recognized_text",
            self.text_callback,
            10
        )

        # 发布语音反馈 → 给 TTS 播报
        self.feedback_pub = self.create_publisher(
            String,
            "/voice_feedback",
            10
        )

        self.get_logger().info("✅ 创始人问答节点已启动")

    def text_callback(self, msg):
        text = msg.data.strip()
        self.get_logger().info(f"识别内容：{text}")

        # ===================== 创始人问题匹配 =====================
        if any(key in text for key in ["谁创立", "谁创造", "谁发明", "创始人", "谁做的"]):
            answer = "这辆工程车的创始人是海南大学二零二二级刘嘉俊"
            self.publish_feedback(answer)

        if any(key in text for key in ["作用","干什么",]):
            answer = "这辆工程车的作用是：用自己的六轴机械臂，夹取特定的物品"
            self.publish_feedback(answer)

    # 只发话题，不调用TTS，绝对不报错
    def publish_feedback(self, text):
        msg = String()
        msg.data = text
        self.feedback_pub.publish(msg)
        self.get_logger().info(f"🤖 回答：{text}")

def main(args=None):
    rclpy.init(args=args)
    node = FounderAnswerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()