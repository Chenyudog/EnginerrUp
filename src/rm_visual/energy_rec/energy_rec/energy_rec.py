import rclpy
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from ultralytics import YOLO
from energy_reco.msg import EnergyRecInterface
from energy_reco.msg import SendMessage

# 不再直接使用 package:// 写法
# yolomodel = "package://energy_rec/best.pt"

class EnergyLatticeDetectNode(Node):
    def __init__(self):
        super().__init__('energy_lattice_recognizer')
        self.depth_image = None
        self.depth_val = 0.0
        
        # 获取 energy_rec 包的安装路径，并构建模型文件的绝对路径
        package_path = get_package_share_directory('energy_rec')
        model_path = os.path.join(package_path, 'best.pt')
        self.get_logger().info(f"Loading model from: {model_path}")
        self.model = YOLO(model_path)
        
        self.bridge = CvBridge()

        # 订阅 RGB 图像
        self.rgb_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.rgb_callback, 10)

        # 订阅深度图像
        self.depth_sub = self.create_subscription(
            Image, "/camera/depth/image_raw", self.depth_callback, 10)
        
        # 发布处理后的图像
        self.img_pub = self.create_publisher(
            Image, "/energy_lattice/result", 10)
        
        # 发布能量单元中心坐标 (2D)
        self.center_position_pub = self.create_publisher(
            EnergyRecInterface, "/energy_lattice/center_pose", 10)
        
        # 订阅中心坐标并获取深度
        self.center_position_sub = self.create_subscription(
            EnergyRecInterface, "/energy_lattice/center_pose",
            self.center_position_callback, 10)

        # 发布能量单元三维坐标
        self.energy_lattice_position_pub = self.create_publisher(
            SendMessage, "/energy_lattice/energy_lattice_position", 10)

        self.get_logger().info("✅ 能量单元识别节点启动")
        
    def rgb_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            results = self.model(cv_image, conf=0.8)
            vis_image = results[0].plot()
            ros_img = self.bridge.cv2_to_imgmsg(vis_image, "bgr8")
            ros_img.header = msg.header
            self.img_pub.publish(ros_img)

            if len(results[0].boxes) > 0:
                cx, cy, width, height = results[0].boxes.xywh[0].cpu().numpy()
                center_msg = EnergyRecInterface()
                center_msg.cx = int(cx)
                center_msg.cy = int(cy)
                self.center_position_pub.publish(center_msg)
                self.get_logger().info(f"发布坐标：cx={int(cx)}, cy={int(cy)}")

        except CvBridgeError as e:
            self.get_logger().error(f"图像转换失败：{e}")
        except Exception as e:
            self.get_logger().error(f"错误：{str(e)}")
            
    def depth_callback(self, msg):
        # 深度图像通常为 16UC1 或 32FC1，根据实际话题类型调整
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
    
    def center_position_callback(self, msg):
        cx = msg.cx
        cy = msg.cy

        if self.depth_image is None:
            self.get_logger().warn("深度图像尚未收到")
            return

        try:
            depth_raw = self.depth_image[cy, cx]  # 注意：深度图像坐标 (row, col)
            self.depth_val = depth_raw

            send_msg = SendMessage()
            send_msg.cx = int(cx)
            send_msg.cy = int(cy)
            send_msg.depth = int(self.depth_val)

            self.energy_lattice_position_pub.publish(send_msg)
            self.get_logger().info(f"发布三维坐标：({cx}, {cy}, {self.depth_val})")
        except IndexError:
            self.get_logger().error(f"坐标 ({cx}, {cy}) 超出深度图像范围")
        except Exception as e:
            self.get_logger().error(f"获取深度失败：{e}")

def main(args=None):
    rclpy.init(args=args)
    node = EnergyLatticeDetectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()