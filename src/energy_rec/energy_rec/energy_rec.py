#1.订阅RGB图像和深度图像
#2.使用YOLO模型进行能量单元识别
#3.发布处理后的图像结果
#4.发布能量单元中心坐标
#5.订阅能量单元中心坐标，并获取该点的深度
#6.发布能量单元的三维坐标



import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from ultralytics import YOLO
from energy_reco.msg import EnergyRecInterface
from energy_reco.msg import SendMessage

yolomodel = "package://energy_rec/best.pt"

class EnergyLatticeDetectNode(Node):
    def __init__(self):
        super().__init__('energy_lattice_recognizer')
        self.depth_image = None
        self.depth_val = 0.0
        self.model = YOLO(yolomodel)
        self.bridge = CvBridge()

##1.订阅RGB图像和深度图像
        # 订阅rgb图像
        self.rgb_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.rgb_callback, 10)

        # 订阅深度图像
        self.depth_sub = self.create_subscription(
            Image, "/camera/depth/image_raw", self.depth_callback, 10)
        
##2.发布模型处理后结果图片
        # 发布模型处理后结果图片
        self.img_pub = self.create_publisher(
            Image, "/energy_lattice/result", 10)
        
##3.发布能量单元中心坐标
        # 发布能量单元中心坐标
        self.center_position_pub = self.create_publisher(
            EnergyRecInterface, "/energy_lattice/center_pose", 10)
        
##4.订阅能量单元中心坐标，并获取该点的深度
        self.center_position_sub = self.create_subscription(
            EnergyRecInterface, "/energy_lattice/center_pose",self.center_position_callback, 10)


##5.发布能量单元坐标话题
        self.energy_lattice_position_pub = self.create_publisher(
            SendMessage, "/energy_lattice/energy_lattice_position", 10)

        self.get_logger().info("✅ 能量单元识别节点启动")
        
    def rgb_callback(self, msg):
        try:
            # 图像转换
            #深度相机发布的图像为ros的格式 这种格式无法被yolo处理 
            # 所以要用cv_bridge转换成opencv的格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 使用yolo模型进行能量单元识别
            results = self.model(cv_image, conf=0.8)

            #画框，可视化结果
            vis_image = results[0].plot()

            #得到中心坐标
            cx,cy,width,height = results[0].boxes.xywh[0].cpu().numpy()

            # 发布图像
            # 将opencv格式的图像转成ros的格式
            ros_img = self.bridge.cv2_to_imgmsg(vis_image, "bgr8")

            ros_img.header = msg.header
            self.img_pub.publish(ros_img)

            if len(results[0].boxes) > 0:
                # 有目标才执行
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
        self.depth_image=self.bridge.imgmsg_to_cv2(msg, "16UC1")
    
    def center_position_callback(self, msg):
        cx = msg.cx
        cy = msg.cy

        if self.depth_image is None:
            return

        try:
            # 获取深度
            depth_raw = self.depth_image[cy, cx]
            self.depth_val = depth_raw 

            # 发布3D坐标
            send_msg = SendMessage()
            send_msg.cx = int(cx)
            send_msg.cy = int(cy)
            send_msg.depth = int(self.depth_val)

            self.energy_lattice_position_pub.publish(send_msg)
        except:
            pass
        
def main(args=None):
    rclpy.init(args=args)
    node = EnergyLatticeDetectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
