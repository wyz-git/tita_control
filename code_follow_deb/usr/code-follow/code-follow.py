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

class JoyPublisher(Node):
    def __init__(self):
        super().__init__('code_follow')
        self.output = self.get_topic_name()
        self.publisher_ = self.create_publisher(Joy, f"/{self.output}/joy", 10)  # 创建一个发布者
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.timer1 = self.create_timer(0.02, self.timer_callback1)
        self.subscription = self.create_subscription(
            PoseArray,
            '/aruco_poses',
            self.listener_callback,
            10  # QoS参数
        )
        self._lock_axes_mode = False
        self._lock_timer = None

        # 订阅 /tita3037207/switch_states
        self.subscription_switch = self.create_subscription(
            String,
            f"/{self.output}/switch_states",
            self.switch_states_callback,
            10
        )

        # 初始化变量
        self.axes3 = 0
        self.axes4 = 0
        self.axes5 = 0
        self.axes6 = -1
        self.axes7 = 0
        self.axes8 = 1

        self.msg_flag = 0
        self.axes_mode = False
        self.c_angle = 0
        self.standard_angle = 480
        self.distance_to_qr_code = 0

    def get_topic_name(self):
        command_str = "ros2 topic list | grep tita | awk -F/ '{print $2}' | head -n 1 | sed 's/\\..*//'"
        try:
            result = subprocess.run(command_str, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            print(f"Command output: {result.stdout}")  # 打印标准输出
            output = result.stdout.strip()
            if not output.startswith('tita'):
                raise ValueError("Extracted topic name does not start with 'tita'.")
            return output
        except subprocess.CalledProcessError as e:
            print(f"Command failed with error: {e.stderr}")  # 打印错误信息
            raise ValueError(f"Error executing command: {e.stderr}")

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
                self.axes3 = 0.1
            else:
                self.axes3 = (0.5 - self.distance_to_qr_code) * 2
            self.axes4 = (self.standard_angle - self.c_angle) * 1.8 / 480

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

def main(args=None):
    rclpy.init(args=args)
    node = JoyPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
        print("程序已终止")

if __name__ == '__main__':
    main()