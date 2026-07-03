import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import os

class RgbPictureCapture(Node):
    def __init__(self):
        super().__init__("rgb_picture_capture_node")

        self.bridge = CvBridge()
        # 获取当前脚本所在文件夹（voice_picture_detect）
        current_file_dir = os.path.dirname(os.path.abspath(__file__))
        
        # 拼接：当前目录 / picture
        self.save_path = "/home/ubuntu/EnginerrUp/src/rm_voice/voice_picture_detect/picture"
        os.makedirs(self.save_path, exist_ok=True)

        # 缓存最新RGB画面
        self.latest_rgb_frame = None

        # 订阅语音识别
        self.text_sub = self.create_subscription(
            String,
            "/recognized_text",
            self.text_callback,
            10
        )

        # 订阅相机RGB
        self.rgb_sub = self.create_subscription(
            Image,
            "camera/color/image_raw",
            self.rgb_callback,
            10
        )

        self.get_logger().info("✅ 语音单张拍照节点启动：每次拍照清空旧图")

    # 持续接收最新画面
    def rgb_callback(self, msg):
        try:
            self.latest_rgb_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {str(e)}")

    # 清空文件夹所有图片
    def clear_all_old_images(self):
        for file_name in os.listdir(self.save_path):
            file_full_path = os.path.join(self.save_path, file_name)
            # 只删除图片文件
            if os.path.isfile(file_full_path):
                os.remove(file_full_path)
        self.get_logger().info("🗑️ 已清空历史所有图片")

    # 语音触发拍照
    def text_callback(self, msg):
        text = msg.data.strip()
        self.get_logger().info(f"识别到内容：{text}")

        # 触发关键词
        if any(key in text for key in ["看到了什么","显示什么"]):
            if self.latest_rgb_frame is None:
                self.get_logger().warn("⚠️ 未获取到相机画面，无法拍照")
                return

            # 1. 先删除所有旧图
            self.clear_all_old_images()
            # 2. 只保存单一张固定名字图片（覆盖式）
            save_file = os.path.join(self.save_path, "picture.png")
            cv2.imwrite(save_file, self.latest_rgb_frame)
            self.get_logger().info(f"📸 拍摄完成，已保存单张图：{save_file}")

def main(args=None):
    rclpy.init(args=args)
    node = RgbPictureCapture()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()