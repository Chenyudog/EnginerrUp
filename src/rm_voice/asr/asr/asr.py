# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import time
import queue
import sounddevice as sd
import numpy as np
import nls
import json
import sys
from .get_token import *
class OnlineASRNode(Node):
    def __init__(self):
        super().__init__('asr_node')
        self.get_logger().info("✅ 语音识别已启动")

        # 阿里云配置
        self.URL = "wss://nls-gateway-cn-shanghai.aliyuncs.com/ws/v1"
        self.TOKEN = token
        self.APPKEY = "JzdfrPXBjvbNsU7c"

        self.publisher_ = self.create_publisher(String, "/recognized_text", 10)
        self.audio_queue = queue.Queue()
        
        # TTS播放状态，用于静音ASR
        self.is_tts_speaking = False
        self.tts_speaking_sub = self.create_subscription(
            Bool,
            '/tts_speaking',
            self.tts_speaking_callback,
            10
        )
        
        self.start_recognizer()

    def audio_callback(self, indata, frames, time, status):
        self.audio_queue.put(indata.copy())

    def tts_speaking_callback(self, msg):
        self.is_tts_speaking = msg.data
        if self.is_tts_speaking:
            self.get_logger().info("🔇 TTS正在播放，ASR暂时静音")
        else:
            self.get_logger().info("🔊 TTS播放结束，ASR恢复监听")

    def on_result(self, message, *args):
        try:
            data = json.loads(message)
            text = data["payload"]["result"]
            
            # 如果TTS正在播放，则不发布识别结果
            if self.is_tts_speaking:
                self.get_logger().info(f"> [已过滤] {text}")
                return
            
            self.get_logger().info(f"> {text}")
            msg = String()
            msg.data = text
            self.publisher_.publish(msg)
        except:
            pass

    def start_recognizer(self):
        self.recognizer = nls.NlsSpeechTranscriber(
            url=self.URL,
            token=self.TOKEN,
            appkey=self.APPKEY,
            on_sentence_end=self.on_result,
        )
        
        self.recognizer.start(
            aformat="pcm",
            sample_rate=16000,
            enable_intermediate_result=False,
            enable_punctuation_prediction=True,
            enable_inverse_text_normalization=True
        )

        self.stream = sd.InputStream(
            callback=self.audio_callback,
            channels=1,
            samplerate=16000,
            dtype='int16'
        )
        self.stream.start()

        self.create_timer(0.01, self.send_audio)

    def send_audio(self):
        while not self.audio_queue.empty():
            data = self.audio_queue.get()
            self.recognizer.send_audio(data.tobytes())

def main(args=None):
    rclpy.init(args=args)
    node = OnlineASRNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()