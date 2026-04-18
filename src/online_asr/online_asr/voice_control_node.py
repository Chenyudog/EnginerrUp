# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# 导入你自定义的消息！！！
from rmctrl_msgs.msg import VoiceControl

import sys
import os

# 添加 python_asr 目录到路径
sys.path.append(os.path.join(os.path.dirname(__file__), '../python_asr'))
from tts import TestTts, pcm2wav, play_audio
import time

#逻辑：
#1.人说话
#2.asr识别成文本
#3.文本发布到 /recognized_text 话题
#4.语音控制节点订阅 /recognized_text 话题，解析指令
#5.根据指令发布 VoiceControl 消息控制底盘
REQUIRE_MODE=0
FORWARD_MODE=1
BACKWARD_MODE=2
LEFT_MODE=3
RIGHT_MODE=4
STOP_MODE=5

#是否有说距离的指令
NO_DISTANCE=0
HAVE_DISTANCE=1
class VoiceControlNode(Node):
    def __init__(self):
        super().__init__('voice_control_node')
        # 初始化状态  为了检测后续语音补充的时候做的标志位
        self.mode=REQUIRE_MODE
        self.distance=NO_DISTANCE
        #创建发布者
        self.subscription = self.create_subscription(
            String,
            '/recognized_text',
            self.listener_callback,
            10
        )
        self.voice_control_chassis_speed_vx=0.0
        self.voice_control_chassis_speed_vy=0.0
        self.voice_control_chassis_speed_vw=0.0
        #创建定时器
        self.timer_=self.create_timer(0.01, self.timer_callback)
        #c
        self.start_time = time.time()
        # 发布 VoiceControl 消息
        self.publisher_ = self.create_publisher(VoiceControl, '/chassis_speed', 10)

        self.feedback_publisher_ = self.create_publisher(String, '/voice_feedback', 10)
        self.get_logger().info('语音控制节点启动')

    def listener_callback(self, msg):
        text = msg.data
        #设置时间戳 为后续是否发布速度作准备（cyd偏爱用时间戳来记录时间）

        self.get_logger().info(f'识别到: {text}')
        
        if '前进' in text or '向前' in text or '直走' in text:
            self.mode = FORWARD_MODE
            self.start_time = time.time()
        elif '后退' in text or '向后' in text:
            self.mode = BACKWARD_MODE
            self.start_time = time.time()
        elif '左移' in text or '向左' in text:
            self.mode = LEFT_MODE
            self.start_time = time.time()

        elif '右移' in text or '向右' in text:
            self.mode = RIGHT_MODE
            self.start_time = time.time()

        elif '停止' in text or '停' in text:
            self.mode = STOP_MODE
            self.start_time = time.time()

        #如果没有识别到动作指令 就询问应该做什么动作
        if self.mode ==REQUIRE_MODE:
            self.publish_feedback('请问需要我做什么动作？')

        #如果没有识别到数字 就询问数字
        self.distance = self.chinese_to_number(text)
        if self.distance <=0:
            self.publish_feedback('请问我应该走多远？')
        else:
            if self.mode is FORWARD_MODE:
                self.voice_control_chassis_speed_vx=1.0
                self.get_logger().info(f'前进 {self.distance} 米')
                self.publish_feedback(f'好的，已执行前进 {self.distance} 米')

            elif self.mode is BACKWARD_MODE:
                self.voice_control_chassis_speed_vx=-1.0
                self.get_logger().info(f'后退 {self.distance} 米')
                self.publish_feedback(f'好的，已执行后退 {self.distance} 米')

            elif self.mode is LEFT_MODE:
                self.voice_control_chassis_speed_vy=1.0
                self.get_logger().info(f'左移 {self.distance} 米')
                self.publish_feedback(f'好的，已执行左移 {self.distance} 米')

            elif self.mode is RIGHT_MODE:
                self.voice_control_chassis_speed_vy=-1.0
                self.get_logger().info(f'右移 {self.distance} 米')
                self.publish_feedback(f'好的，已执行右移 {self.distance} 米')
            elif self.mode is STOP_MODE:
                self.voice_control_chassis_speed_vx = 0.0
                self.voice_control_chassis_speed_vy = 0.0
                self.voice_control_chassis_speed_vw = 0.0
                self.publish_feedback("已停止")

    
    def publish_feedback(self, feedback):
        msg = String()
        msg.data = feedback
        self.feedback_publisher_.publish(msg)
        self.get_logger().info(f'反馈: {feedback}')
        # 使用阿里云 TTS 进行语音反馈
        self.speak_with_aliyun_tts(feedback)
    
    def speak_with_aliyun_tts(self, text):
        try:
            # 临时文件路径
            pcm_path = os.path.join(os.path.dirname(__file__), '../python_asr', 'temp_tts.pcm')
            wav_path = os.path.join(os.path.dirname(__file__), '../python_asr', 'temp_tts.wav')
            
            # 确保目录存在
            os.makedirs(os.path.dirname(pcm_path), exist_ok=True)
            
            # 开始语音合成
            self.get_logger().info(f'正在合成语音: {text}')
            t = TestTts("tts", pcm_path)
            t.start(text)
            
            # 等待合成结束
            time.sleep(1)
            
            # 转换为 WAV 并播放
            pcm2wav(pcm_path, wav_path)
            self.get_logger().info('正在播放反馈语音')
            play_audio(wav_path)
            
            # 清理临时文件
            if os.path.exists(pcm_path):
                os.remove(pcm_path)
            if os.path.exists(wav_path):
                os.remove(wav_path)
                
        except Exception as e:
            self.get_logger().error(f'语音合成失败: {str(e)}')

    def timer_callback(self):
            msg=VoiceControl()
            self.current_time = time.time()
            if self.current_time - self.start_time < self.distance:
                msg.vx = self.voice_control_chassis_speed_vx
                msg.vy = self.voice_control_chassis_speed_vy
                msg.vw = self.voice_control_chassis_speed_vw
            else:
                msg.vx = 0.0
                msg.vy = 0.0
                msg.vw = 0.0
            self.publisher_.publish(msg)

    #提取数字函数
    # 从语音文本中提取数字（例如：前进5米 → 得到 5.0）
    def extract_number(self, text):
        result = ''
        for char in text:
            if char.isdigit() or char == '.':
                result += char
        if result:
            return float(result)
        return None
    
    #中文转阿拉伯数字函数
    def chinese_to_number(self, text):
        chinese_numerals = {
            '零': 0, '一': 1, '二': 2, '三': 3, '四': 4,
            '五': 5, '六': 6, '七': 7, '八': 8, '九': 9,
            '十': 10, '百': 100, '千': 1000, '万': 10000
        }
        result = 0
        current_unit = 1
        for char in reversed(text):
            if char in chinese_numerals:
                value = chinese_numerals[char]
                if value >= 10:
                    current_unit = value
                else:
                    result += value * current_unit
            else:
                continue
        return result
def main(args=None):
    rclpy.init(args=args)
    node = VoiceControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()