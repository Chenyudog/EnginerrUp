import rclpy
from rclpy.node import Node
class RgbPictureCapture(Node):
    def __init__(self):
        super().__init__("rgb_voice_capture_node")
        self.


def main():
    rclpy.init()
    node=Node(RgbPictureCapture)
    rcly.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()