import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, Image
from cv_bridge import CvBridge
import mediapipe as mp
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy
import cv2
import subprocess
from std_msgs.msg import String
import json

class JoyPublisher(Node):
    def __init__(self):
        super().__init__('joy_gesture_controller')
        
        # 控制状态初始化
        self.control_state = {
            'axes': [0.0] * 9,  # axes[0]-axes[8]
            'mode': 1           # 控制模式 (0:手势模式)
        }
        self.output = self.get_topic_name()
        # 图像处理相关初始化
        self.bridge = CvBridge()
        self.current_frame = None
        
        # MediaPipe手势识别
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.5
        )
        self.mp_drawing = mp.solutions.drawing_utils
        qos_policy = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        self.kp = 0.8  # 比例系数
        self.ki = 0.00 #积分系数
        self.kd = 0.1  # 微分系数
        self.standard_angle = 480
        self.prev_error = 0
        self.integral = 0
        self.axes_mode = False
        self.control_state['axes'][5] = -1.0
        self.control_state['axes'][7] = 1.0
        # ROS2订阅者/发布者
        self.image_sub = self.create_subscription(
            Image,
            f"/{self.output}/perception/camera/image/left",
            self.image_callback,
            qos_policy
        )
        self.subscription_switch = self.create_subscription(
            String,
            f"/{self.output}/switch_states",
            self.switch_states_callback,
            10
        )
        self.joy_pub = self.create_publisher(Joy, f"/{self.output}/joy", 10)
        
        # 控制消息定时发布
        self.create_timer(0.02, self.publish_control)

        # 初始化日志级别
        self.declare_parameter('log_level', 20)  # 默认INFO级别
        self.log_level = self.get_parameter('log_level').value
        self.get_logger().set_level(self.log_level)

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
            self.axes_mode = switch_states.get("5", 0)  # 如果不存在 "4"，默认值为 0
            #self.get_logger().info(f"Updated axes_mode: {self.axes_mode}")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse switch_states message: {e}")

    def image_callback(self, msg):
        """图像订阅回调函数"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # self.get_logger().debug("收到新图像帧", throttle_duration_sec=1.0)
                
            if self.control_state['mode'] == 1:
                self.process_gesture(cv_image)
                
        except Exception as e:
            self.get_logger().error(f"图像处理异常: {str(e)}")

    def process_gesture(self, frame):
        """手势识别处理"""
        # self.get_logger().debug("开始手势识别处理")
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)
        
        if results.multi_hand_landmarks:
            # self.get_logger().info("检测到手势", throttle_duration_sec=0.5)
            self.update_gesture_controls(results, frame.shape)
        else:
            # self.get_logger().debug("未检测到手部")
            self.reset_controls()

    def update_gesture_controls(self, results, frame_shape):
        """更新手势控制状态"""
        height, width = frame_shape[:2]
        image_center = (width // 2, height // 2)
        
        for idx, hand_landmarks in enumerate(results.multi_hand_landmarks):
            # self.get_logger().debug(f"处理第 {idx+1} 只手的手势")
            
            # 获取手腕位置
            wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
            wrist_pos = (int(wrist.x * width), int(wrist.y * height))
            
            # 记录手腕相对位置
            # self.get_logger().debug(f"手腕坐标 X:{wrist_pos[0]} Y:{wrist_pos[1]}")

            # 水平控制
            prev_h = self.control_state['axes'][3]
            new_h = self.calculate_horizontal_control(wrist_pos[0], image_center[0], width)
            self.control_state['axes'][3] = new_h
            # if prev_h != new_h:
            #     self.get_logger().info(f"水平控制量变化: {prev_h} → {new_h}")

            # 垂直控制
            prev_v = self.control_state['axes'][2]
            new_v = self.calculate_vertical_control(hand_landmarks)
            self.control_state['axes'][2] = new_v
            # if prev_v != new_v:
            #     self.get_logger().info(f"垂直控制量变化: {prev_v} → {new_v}")

    def calculate_horizontal_control(self, x, center_x, frame_width):
        """计算水平方向控制量"""
        dead_zone = frame_width * 0.1
        delta = x - center_x
        
        if abs(delta) < dead_zone:
            # self.get_logger().debug(f"处于死区范围: {abs(delta):.1f} < {dead_zone:.1f}")
            return 0.0
            
        direction = "左" if delta < 0 else "右"
        # self.get_logger().info(f"水平方向: {direction} 偏移量: {abs(delta):.1f}px")
        self.integral += delta
        derivative = delta - self.prev_error
        self.prev_error = delta
        axes4 = (self.kp * delta + self.ki * self.integral + self.kd * derivative) * 1.8 / 480
        return -axes4

    def calculate_vertical_control(self, landmarks):
        """计算垂直方向控制量"""
        index_tip = landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        wrist = landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
        
        distance = np.linalg.norm([
            index_tip.x - wrist.x,
            index_tip.y - wrist.y
        ])
        if distance < 0.15:
            control_value = -1.0
        elif distance > 0.5:
            control_value = 0.5
        else :
            control_value = 0.0
        # self.get_logger().info(f"距离: {distance:.3f}")
        return control_value

    def reset_controls(self):
        """重置控制状态"""
        if self.control_state['axes'][2] != 0 or self.control_state['axes'][3] != 0:
            # self.get_logger().warning("手势丢失，正在重置控制状态")
            self.control_state['axes'][2] = 0.0
            self.control_state['axes'][3] = 0.0

    def publish_control(self):
        if self.axes_mode == True:
            """发布控制消息"""
            joy_msg = Joy()
            joy_msg.header.stamp = self.get_clock().now().to_msg()
            joy_msg.header.frame_id = "gesture_control"
            joy_msg.axes = self.control_state['axes']
            self.joy_pub.publish(joy_msg)
            # self.get_logger().debug("已发布控制指令", throttle_duration_sec=0.1)

def main(args=None):
    rclpy.init(args=args)
    controller = JoyPublisher()
    
    try:
        controller.get_logger().info("手势控制节点已启动")
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("关闭节点...")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()