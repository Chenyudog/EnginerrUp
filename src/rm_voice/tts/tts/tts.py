# -*- coding: utf-8 -*-
import nls
import time
import threading
import os
import wave
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ===================== 阿里云配置 =====================
URL = "wss://nls-gateway-cn-shanghai.aliyuncs.com/ws/v1"
TOKEN = "3e35c5af174242259d0a8d5ccd3dabe3"
APPKEY = "zvNV8ngJlGovddWG"

# ===================== 工具函数 =====================
def pcm2wav(pcm_path, wav_path):
    with open(pcm_path, 'rb') as pcmfile:
        pcmdata = pcmfile.read()
    with wave.open(wav_path, 'wb') as wavfile:
        wavfile.setparams((1, 2, 16000, 0, 'NONE', 'NONE'))
        wavfile.writeframes(pcmdata)

def play_audio(wav_path):
    subprocess.run(["aplay", wav_path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

# ===================== 阿里云TTS核心类 =====================
class TestTts:
    def __init__(self, tid, test_file):
        self.__th = threading.Thread(target=self.__test_run)
        self.__id = tid
        self.__test_file = test_file

    def start(self, text):
        self.__text = text
        output_dir = os.path.dirname(self.__test_file)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir)
        self.__f = open(self.__test_file, "wb")
        self.__th.start()

    def test_on_metainfo(self, message, *args):
        pass

    def test_on_error(self, message, *args):
        print("on_error:", message, args)

    def test_on_close(self, *args):
        try:
            self.__f.close()
        except:
            pass

    def test_on_data(self, data, *args):
        try:
            self.__f.write(data)
        except:
            pass



    def __test_run(self):
        tts = nls.NlsSpeechSynthesizer(
            url=URL,
            token=TOKEN,
            appkey=APPKEY,
            on_metainfo=self.test_on_metainfo,
            on_data=self.test_on_data,
            on_error=self.test_on_error,
            on_close=self.test_on_close,
            callback_args=[self.__id]
        )
        tts.start(self.__text, voice="ailun")

# ===================== ROS2 TTS朗读节点 =====================
class TTS(Node):
    def __init__(self):
        super().__init__('tts_node')

        # 订阅/base_control 话题
        self.subscription = self.create_subscription(
            String,
            "/base_control",
            self.tts_callback,
            10
        )

        self.picture_description_subscription = self.create_subscription(
            String,
            "/picture_description",
            self.tts_picture_description_callback,
            10
        )


        self.engineer_info_subscription = self.create_subscription(
            String,
            "/voice_feedback",
            self.engineer_info_callback,
            10
        )
        self.get_logger().info("✅ TTS 语音播报节点已启动，等待消息...")

    def tts_callback(self, msg):
        text = msg.data
        self.get_logger().info(f"📢 收到播报：{text}")

        try:
            # 临时文件路径
            pcm_path = os.path.join(os.path.dirname(__file__), 'temp_tts.pcm')
            wav_path = os.path.join(os.path.dirname(__file__), 'temp_tts.wav')

            # 合成语音
            t = TestTts("tts", pcm_path)
            t.start(text)

            # 等待合成
            time.sleep(2)

            # 转格式 + 播放
            pcm2wav(pcm_path, wav_path)
            play_audio(wav_path)

            # 删除临时文件
            if os.path.exists(pcm_path):
                os.remove(pcm_path)
            if os.path.exists(wav_path):
                os.remove(wav_path)

        except Exception as e:
            self.get_logger().error(f"播放失败：{str(e)}")

    def tts_picture_description_callback(self, msg):
        text = msg.data
        self.get_logger().info(f"📢 收到播报：{text}")

        try:
            # 临时文件路径
            pcm_path = os.path.join(os.path.dirname(__file__), 'temp_tts.pcm')
            wav_path = os.path.join(os.path.dirname(__file__), 'temp_tts.wav')

            # 合成语音
            t = TestTts("tts", pcm_path)
            t.start(text)

            # 等待合成
            time.sleep(5)

            # 转格式 + 播放
            pcm2wav(pcm_path, wav_path)
            play_audio(wav_path)

            # 删除临时文件
            if os.path.exists(pcm_path):
                os.remove(pcm_path)
            if os.path.exists(wav_path):
                os.remove(wav_path)

        except Exception as e:
            self.get_logger().error(f"播放失败：{str(e)}")

    def engineer_info_callback(self,msg):
        text = msg.data
        self.get_logger().info(f"📢 收到播报：{text}")

        try:
            # 临时文件路径
            pcm_path = os.path.join(os.path.dirname(__file__), 'temp_tts.pcm')
            wav_path = os.path.join(os.path.dirname(__file__), 'temp_tts.wav')

            # 合成语音
            t = TestTts("tts", pcm_path)
            t.start(text)

            # 等待合成
            time.sleep(2)

            # 转格式 + 播放
            pcm2wav(pcm_path, wav_path)
            play_audio(wav_path)

            # 删除临时文件
            if os.path.exists(pcm_path):
                os.remove(pcm_path)
            if os.path.exists(wav_path):
                os.remove(wav_path)

        except Exception as e:
            self.get_logger().error(f"播放失败：{str(e)}")
# ===================== 主函数 =====================
def main(args=None):
    rclpy.init(args=args)
    node = TTS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()