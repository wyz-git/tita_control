import subprocess
import threading
import time
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import pyzbar.pyzbar as pyzbar
import math
from geometry_msgs.msg import PoseArray
import transforms3d as t3d
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class JoyPublisher(Node):
    def __init__(self):
        super().__init__('code_follow')
        self.output = self.get_topic_name()
        self.publisher_ = self.create_publisher(Joy, f"/{self.output}/joy", 10)  # 创建一个发布者
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.bridge = CvBridge()
        self.timer1 = self.create_timer(0.02, self.timer_callback1)
        self.subscription = self.create_subscription(
            PoseArray,
            '/aruco_poses',
            self.listener_callback,
            10  # QoS参数
        )
        self._lock_axes_mode = False
        self._lock_timer = None
        self.subscription1 = self.create_subscription(
            Joy,
            f"/{self.output}/joy",
            self.listener_callback1,
            10
        )
        self.subscription1  # 防止被垃圾回收

        # 初始化变量
        self.axes1 = 0
        self.axes2 = 0
        self.axes3 = 0
        self.axes4 = 0
        self.axes5 = 0
        self.axes6 = -1
        self.axes7 = 0
        self.axes8 = 1

        self.z_value = 0
        self.msg_flag = 2
        self.axes_mode = 1
        self.step = 0
        self.stop_time = 0
        self.c_angle = 0
        self.standard_step = 0.35
        self.standard_step_limit = 0.06
        self.diagonal_distance = 0
        self.standard_distance = 0.76
        self.standard_angle = 480
        self.euler_angles = None
        self.distance_to_qr_code1 = 0
        self.distance_to_qr_code2 = 0

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

    def listener_callback1(self, msg):
        if len(msg.axes) > 8:
            if self.stop_time <= 1000:
                self.stop_time += 1
            if msg.axes[8] == 4 and not self._lock_axes_mode:
                self._lock_axes_mode = True
                self.axes_mode = 1 if self.axes_mode == 0 else 0
                self.stop_time = 0
                if self._lock_timer is not None:
                    self._lock_timer.cancel()
                self._lock_timer = threading.Timer(1.0, self.unlock_wrapper)
                self._lock_timer.start()
        if self.stop_time > 1000:
            self.axes_mode = 1

    def unlock_axes_mode(self):
        self._lock_axes_mode = False

    def unlock_wrapper(self):
        self.unlock_axes_mode()

    def listener_callback(self, msg):
        self.msg_flag = 1
        self.c_angle = int(msg.header.frame_id.split('_')[0])
        c_id = int(msg.header.frame_id.split('_')[-1])
        if c_id == 2:
            for pose in msg.poses:
                position = pose.position
                orientation = pose.orientation
                self.distance_to_qr_code1 = 0.55 * math.sqrt(position.x**2 + position.y**2 + position.z**2)
                self.z_value = orientation.z
                if orientation.z / orientation.x > 0:
                    self.z_value = abs(self.z_value)
                else:
                    self.z_value = -abs(self.z_value)
            quaternion = [orientation.w, orientation.x, orientation.y, orientation.z]
            rotation_matrix = t3d.quaternions.quat2mat(quaternion)
            self.euler_angles = t3d.euler.mat2euler(rotation_matrix, axes='sxyz')

    def decode_qr_code(self):
        if self.axes_mode == 1:
            self.step = 0
            return

        if self.msg_flag == 1:
            if self.distance_to_qr_code1 < 0.50:
                self.axes3 = 0.1
            else:
                self.axes3 = (0.5 - self.distance_to_qr_code1) * 2
            self.axes4 = (self.standard_angle - self.c_angle) * 1.8 / 480

        elif self.msg_flag > 50:
            self.axes3 = 0.0
            if self.axes4 < 0:
                self.axes4 = -0.2
            if self.axes4 > 0:
                self.axes4 = 0.2

        if self.msg_flag < 1000:
            self.msg_flag += 1

    def decodeDisplay(self, image):
        lista = []
        barcodes = pyzbar.decode(image)
        for barcode in barcodes:
            (y, x, w, h) = barcode.rect
            print("x:{} y:{} w:{} h:{}".format(x, y, w, h))
            centerRotation = [w / 2, h / 2]
            barcodeData = barcode.data.decode("utf-8")
            barcodeType = barcode.type
            print("[INFO] Found {} barcode: {}".format(barcodeType, barcodeData))
            thresh1 = image[x:x + h, y:y + w]
            GrayImage = cv2.cvtColor(thresh1, cv2.COLOR_BGR2GRAY)
            ret, frame = cv2.cv2.threshold(GrayImage, 127, 255, cv2.THRESH_BINARY)
            for pix_x in range(5, h - 5):
                for pix_y in range(5, w - 5):
                    nb = 0
                    for row in range(pix_x - 5, pix_x + 5):
                        for clo in range(pix_y - 5, pix_y + 5):
                            gery = frame[row][clo]
                            if gery == 0:
                                nb += 1
                    if nb > 99:
                        lista.append([pix_y, pix_x])
        print(lista)
        return image, lista, centerRotation

    def intercept(self, a, b):
        selfx = a[0] - b[0]
        selfy = a[1] - b[1]
        selflen = math.sqrt((selfx**2) + (selfy**2))
        return selflen

    def SeekingRightAngles(self, lista):
        a = self.intercept(lista[0], lista[1])
        b = self.intercept(lista[1], lista[2])
        c = self.intercept(lista[2], lista[0])
        A = math.degrees(math.acos((a * a - b * b - c * c) / (-2 * b * c)))
        B = math.degrees(math.acos((b * b - a * a - c * c) / (-2 * a * c)))
        C = math.degrees(math.acos((c * c - a * a - b * b) / (-2 * a * b)))
        print(A)
        print(B)
        print(C)
        if A > B and A > C:
            return lista[2]
        if B > A and B > C:
            return lista[0]
        if C > B and C > A:
            return lista[1]

    def RotationAngle(self, rightpoint, centerRotation):
        lista = [[0, 0], rightpoint, centerRotation]
        slope = rightpoint[1] / rightpoint[0]
        if slope == 1:
            if rightpoint[0] < centerRotation[0] / 2:
                return 0
            else:
                return 180
        else:
            a = self.intercept(lista[0], lista[1])
            b = self.intercept(lista[1], lista[2])
            c = self.intercept(lista[2], lista[0])
            A = math.degrees(math.acos((a * a - b * b - c * c) / (-2 * b * c)))
            if slope < 1:
                return -A
            if slope > 1:
                return A

    def timer_callback1(self):
        try:
            self.decode_qr_code()
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")

    def timer_callback(self):
        if self.axes_mode == 1:
            return
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
        process.terminate()
        reader_thread.join()
        print("程序已终止")

if __name__ == '__main__':
    main()