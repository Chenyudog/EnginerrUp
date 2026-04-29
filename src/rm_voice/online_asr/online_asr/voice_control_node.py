# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist  # 这里修好了！
from rmctrl_msgs.msg import VoiceControl
import re

REQUIRE_MODE = 0
FORWARD_MODE = 1
BACKWARD_MODE = 2
LEFT_MODE = 3
RIGHT_MODE = 4
STOP_MODE = 5

class VoiceControlNode(Node):
    def __init__(self):
        super().__init__('voice_control_node')
        
        # 状态
        self.mode = REQUIRE_MODE
        self.distance = 0.0
        self.moving = False

        # 速度
        self.vx = 0.0
        self.vy = 0.0

        # 运动参数
        self.speed = 0.5
        
        # ====================== 数帧专用变量 ======================
        self.need_ticks = 0   # 需要跑多少帧
        self.run_ticks = 0    # 已经跑了多少帧
        self.TICK_RATE = 0.05 # 定时器周期 50ms

        # 订阅语音识别
        self.subscription = self.create_subscription(
            String,
            '/recognized_text',
            self.listener_callback,
            10
        )

        # 发布器
        self.publisher_chassis = self.create_publisher(VoiceControl, '/chassis_speed', 10)
        self.publisher_turtle = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.feedback_publisher = self.create_publisher(String, '/base_control', 10)

        # 定时器
        self.timer_ = self.create_timer(self.TICK_RATE, self.timer_callback)
        self.get_logger().info('✅ 语音控制启动（帧数精准版）')

    def listener_callback(self, msg):
        text = msg.data.strip()
        self.get_logger().info(f'识别到: {text}')

        self.distance = self.parse_distance(text)

        # 动作指令
        if '前进' in text or '向前' in text or '直走' in text:
            self.mode = FORWARD_MODE
        elif '后退' in text or '向后' in text:
            self.mode = BACKWARD_MODE
        elif '左移' in text or '向左' in text:
            self.mode = LEFT_MODE
        elif '右移' in text or '向右' in text:
            self.mode = RIGHT_MODE
        elif '停止' in text or '停' in text:
            self.mode = STOP_MODE
            self.stop_all()
            self.publish_feedback("已停止")
            return
        else:
            self.mode = REQUIRE_MODE
            return

        if self.distance <= 0:
            self.publish_feedback("请问我应该走多远？")
            return

        # ====================== 【核心：纯数帧，不用时间】 ======================
        total_seconds = self.distance / self.speed
        self.need_ticks = int(total_seconds / self.TICK_RATE)
        self.run_ticks = 0
        self.moving = True

        # 设置速度
        if self.mode == FORWARD_MODE:
            self.vx = self.speed
            self.vy = 0.0
            self.publish_feedback(f'前进 {self.distance} 米')

        elif self.mode == BACKWARD_MODE:
            self.vx = -self.speed
            self.vy = 0.0
            self.publish_feedback(f'后退 {self.distance} 米')

        elif self.mode == LEFT_MODE:
            self.vx = 0.0
            self.vy = self.speed
            self.publish_feedback(f'左移 {self.distance} 米')

        elif self.mode == RIGHT_MODE:
            self.vx = 0.0
            self.vy = -self.speed
            self.publish_feedback(f'右移 {self.distance} 米')

    
    def timer_callback(self):
        msg_chassis = VoiceControl()
        msg_turtle = Twist()

        # ====================== 【帧数判断】 ======================
        if self.moving:
            if self.run_ticks < self.need_ticks:
                msg_chassis.vx = self.vx
                msg_chassis.vy = self.vy
                msg_turtle.linear.x = self.vx
                msg_turtle.linear.y = self.vy
                self.run_ticks += 1
            else:
                msg_chassis.vx = 0.0
                msg_chassis.vy = 0.0
                msg_turtle.linear.x = 0.0
                msg_turtle.linear.y = 0.0
                self.moving = False
        else:
            msg_chassis.vx = 0.0
            msg_chassis.vy = 0.0
            msg_turtle.linear.x = 0.0
            msg_turtle.linear.y = 0.0

        self.publisher_chassis.publish(msg_chassis)
        self.publisher_turtle.publish(msg_turtle)

    # 立刻停止所有
    def stop_all(self):
        self.vx = 0.0
        self.vy = 0.0
        self.moving = False
        self.need_ticks = 0
        self.run_ticks = 0

    def publish_feedback(self, feedback):
        msg = String()
        msg.data = feedback
        self.feedback_publisher.publish(msg)
        self.get_logger().info(f'反馈: {feedback}')
 



    # 数字解析
    def parse_distance(self, text):
        num_list = []
        nums = re.findall(r'(\d+\.?\d*)', text)
        for n in nums:
            num_list.append(float(n))
        cn_num = self.chinese_to_number(text)
        if cn_num > 0:
            num_list.append(cn_num)
        if num_list:
            return max(num_list)
        return 0.0

    def chinese_to_number(self, text):
        digit_map = {'零':0,'一':1,'二':2,'两':2,'三':3,'四':4,'五':5,'六':6,'七':7,'八':8,'九':9,'十':10}
        total = temp = 0
        for c in text:
            if c in digit_map:
                val = digit_map[c]
                if val >= 10:
                    total += (temp if temp else 1) * val
                    temp = 0
                else:
                    temp = val
        total += temp
        return total

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