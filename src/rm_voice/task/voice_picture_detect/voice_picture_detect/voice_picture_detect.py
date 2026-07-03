import os
from dashscope import MultiModalConversation
import dashscope 
import rclpy
from std_msgs.msg import String  
from rclpy.node import Node

local_path = "/home/ubuntu/EnginerrUp/src/rm_voice/voice_picture_detect/picture/picture.png"
picture_dir="/home/ubuntu/EnginerrUp/src/rm_voice/voice_picture_detect/picture"
image_path = f"file://{local_path}"
class VoicePictureCapture(Node):
    def __init__(self):
        super().__init__("voice_picture_capture_node")
        self.publisher_=self.create_publisher(String,"picture_description",10)
        # 每秒检查一次图片
        self.timer = self.create_timer(0.3, self.callAPI)



    def callAPI(self):
        if not os.path.exists(local_path):
            self.get_logger().info("未检测到图片 退出程序")
            return
        else:
            #-------------------以下为调用官方api---------------------
            self.get_logger().info("检测到图片 开始识别")
            dashscope.base_http_api_url = "https://dashscope.aliyuncs.com/api/v1"

            messages = [
                        {'role':'user',
                        'content': [{'image': image_path},
                                    {'text': '用一句话概括一下 你看到了什么?'}]}]

            response = MultiModalConversation.call(
                # 各地域的API Key不同。获取API Key：https://help.aliyun.com/zh/model-studio/get-api-key
                api_key="sk-c7f656b80c1149c98968bec7638ef23e",
                model='qwen-vl-plus',  # 此处以qwen-vl-plus为例，可按需更换模型名称。模型列表：https://help.aliyun.com/zh/model-studio/models
                messages=messages)
            result=response.output.choices[0].message.content[0]["text"]

            self.get_logger().info(result)

            #----------------------调用api结束-------------------
            msg=String()
            msg.data=result
            self.publisher_.publish(msg)
            os.remove(local_path)

def main():
    rclpy.init()
    node=VoicePictureCapture()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()
