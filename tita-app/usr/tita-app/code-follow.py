import subprocess
import threading
import time
import math
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
import transforms3d as t3d
from pathlib import Path

class JoyPublisher(Node):
    def __init__(self):
        super().__init__('code_follow')
        # 加载配置文件
        self.config = self.load_config()
        
        # ROS2 配置
        self.publisher_ = self.create_publisher(Joy, f"/{self.config['mqtt_username']}/joy", 10)
        self.mqtt_topic = f"{self.config['mqtt_username']}/virtual"

        self.timer = self.create_timer(0.02, self.timer_callback)
        self.timer1 = self.create_timer(0.02, self.timer_callback1)
        self.subscription = self.create_subscription(
            PoseArray,
            '/aruco_poses',
            self.listener_callback,
            10  # QoS参数
        )

        # 订阅 /tita3037207/switch_states
        self.subscription_switch = self.create_subscription(
            String,
            f"{self.config['mqtt_username']}/switch_states",
            self.switch_states_callback,
            10
        )

        # 初始化变量
        self.axes3 = 0
        self.axes4 = 0
        self.axes5 = -1
        self.axes6 = -1
        self.axes7 = 0
        self.axes8 = 1
        self.kp = 0.8  # 比例系数
        self.ki = 0.00 #积分系数
        self.kd = 0.1  # 微分系数
        self.prev_error = 0
        self.integral = 0
        

        self.msg_flag = 0
        self.axes_mode_last = False
        self.axes_mode = False
        self.c_angle = 0
        self.standard_angle = 480
        self.distance_to_qr_code = 0

    def load_config(self):
        """加载配置文件"""
        config_paths = [
            Path.home() / ".config/mqtt_joy_bridge/config.json",  # 用户级配置
            Path("config.json")  # 当前目录配置
        ]
        
        for path in config_paths:
            if path.exists():
                try:
                    with open(path, "r") as f:
                        config = json.load(f)
                        if "mqtt_username" not in config or "mqtt_password" not in config:
                            raise ValueError("配置文件缺少必要字段")
                        return config
                except Exception as e:
                    self.get_logger().error(f"配置文件加载失败 {path}: {str(e)}")
                    raise
        raise FileNotFoundError("未找到有效的配置文件")
    
    def switch_states_callback(self, msg):
        """订阅 /tita3037207/switch_states 的回调函数"""
        try:
            # 解析 JSON 消息
            switch_states = json.loads(msg.data)
            # 获取 "4" 后面的值
            self.axes_mode = switch_states.get("4", 0)  # 如果不存在 "4"，默认值为 0
            #self.get_logger().info(f"Updated axes_mode: {self.axes_mode}")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse switch_states message: {e}")

    def listener_callback(self, msg):
        self.msg_flag = 1
        self.c_angle = int(msg.header.frame_id.split('_')[0])
        c_id = int(msg.header.frame_id.split('_')[-1])
        if c_id == 2:
            for pose in msg.poses:
                position = pose.position
                orientation = pose.orientation
                self.distance_to_qr_code = 0.55 * math.sqrt(position.x**2 + position.y**2 + position.z**2)
            quaternion = [orientation.w, orientation.x, orientation.y, orientation.z]
            rotation_matrix = t3d.quaternions.quat2mat(quaternion)

    def decode_qr_code(self):
        if self.axes_mode == False:
            return

        if self.msg_flag == 1:
            if self.distance_to_qr_code < 0.50:
                self.axes3 = 0.15
            else:
                self.axes3 = (0.5 - self.distance_to_qr_code) * 3
                # 在decode_qr_code中实现PID
                error = self.standard_angle - self.c_angle
                self.integral += error
                derivative = error - self.prev_error
                self.prev_error = error
                self.axes4 = (self.kp * error + self.ki * self.integral + self.kd * derivative) * 1.8 / 480

        elif self.msg_flag > 50:
            self.axes3 = 0.0
            if self.axes4 < 0:
                self.axes4 = -0.2
            if self.axes4 > 0:
                self.axes4 = 0.2

        if self.msg_flag < 1000:
            self.msg_flag += 1

    def timer_callback1(self):
        try:
            self.decode_qr_code()
        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")

    def timer_callback(self):
        if self.axes_mode_last == True and self.axes_mode == False:
            joy_msg = Joy()
            joy_msg.header.frame_id = "joy"
            joy_msg.axes = [
                float(0),
                float(0),
                float(0),
                float(0),
                float(1),
                float(self.axes6),
                float(self.axes7),
                float(self.axes8),
            ]
            self.publisher_.publish(joy_msg)

        if self.axes_mode == True:
            joy_msg = Joy()
            joy_msg.header.frame_id = "joy"
            joy_msg.axes = [
                float(0),
                float(0),
                float(self.axes3),
                float(self.axes4),
                float(self.axes5),
                float(self.axes6),
                float(self.axes7),
                float(self.axes8),
            ]
            self.publisher_.publish(joy_msg)
        self.axes_mode_last = self.axes_mode

def main(args=None):
    rclpy.init(args=args)
    node = JoyPublisher()

    try:
        node.get_logger().info("标签控制节点已启动")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
        print("程序已终止")

if __name__ == '__main__':
    main()
