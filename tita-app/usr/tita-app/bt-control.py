#!/usr/bin/env python3
import re
import rclpy
import json
import os
import socket
import struct
from pathlib import Path
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import threading  

class BluetoothJoyBridge(Node):
    def __init__(self):
        super().__init__('bluetooth_joy_bridge')
        
        # 加载配置文件
        self.config = self.load_config()
        
        # ROS2 配置
        self.publisher_ = self.create_publisher(Joy, f"/{self.config['mqtt_username']}/joy", 10)
        self.publisher = self.create_publisher(String, f"/{self.config['mqtt_username']}/switch_states", 10)
        # 初始化 axes 数组
        self.axes = [0.0] * 8
        
        # 蓝牙HCI Socket配置
        self.hci_sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_RAW, socket.BTPROTO_HCI)
        self.hci_sock.bind((0,))  # 绑定到第一个蓝牙适配器
        
        # 设置HCI过滤器
        hci_filter = struct.pack(
            "<H8I",
            0xFFFF,  # 类型掩码（ACL + SCO + Command + Event）
            0x00000000,
            0x00000000,
            0x00000000,
            0x00000000,
            0x00000000,
            0x00000000,
            0x00000000,
            0x00000000
        )
        self.hci_sock.setsockopt(socket.SOL_HCI, socket.HCI_FILTER, hci_filter)
        
        # 启动蓝牙数据接收线程
        self.bluetooth_thread = threading.Thread(target=self.receive_bluetooth_data)
        self.bluetooth_thread.daemon = True
        self.bluetooth_thread.start()
        
        self.get_logger().info("蓝牙Joy桥接器已启动，等待数据...")

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

    def receive_bluetooth_data(self):
        """持续接收蓝牙数据"""
        while True:
            data = self.hci_sock.recv(1024)
            buffer = b""
            buffer += data
            if buffer[:3] == bytes.fromhex("020021"):
                # print("ACL 数据包:", data.hex())
                payload = data[12:]  # 根据实际协议调整偏移量
                try:
                    # 尝试解码为UTF-8字符串
                    payload_str = payload.decode('utf-8').strip()
                    # print(f"接收到控制指令: {payload_str}")
                    
                    # 处理控制指令
                    if payload_str.startswith("CTRL:"):
                        self.process_control_data(payload_str)
                    elif payload_str.lstrip().startswith('{"1":'):
                        self.process_switch_data(payload_str)
                except UnicodeDecodeError:
                    print(f"二进制负载: {payload.hex()}")

    def process_switch_data(self, payload_str):
        """处理开关状态JSON数据并发布到ROS2"""
        try:
            # 1. 去除首尾空白字符
            payload_str = payload_str.strip()
            
            # 2. 验证是否为有效JSON（可选步骤）
            try:
                json.loads(payload_str)  # 仅验证不保存结果
            except json.JSONDecodeError as e:
                raise ValueError(f"Invalid JSON format: {str(e)}")
            
            # 3. 按照on_mqtt_message的方式发布
            ros2_msg = String()
            ros2_msg.data = payload_str  # 使用原始JSON字符串
            
            self.publisher.publish(ros2_msg)
            # self.get_logger().info(f"发布开关状态到ROS2: {payload_str}")
            
        except Exception as e:
            self.get_logger().error(f"处理开关数据失败: {str(e)}")
            
            # 发布错误信息（保持与MQTT处理一致）
            error_msg = String()
            error_msg.data = json.dumps({"error": str(e), "raw_data": payload_str})
            self.publisher.publish(error_msg)

    def process_control_data(self, payload_str):
        try:
            # 初始化数据结构
            axes = [0.0] * 8

            ##################################################
            # 解析 CTRL: 格式的数据
            ##################################################
            if not payload_str.startswith("CTRL:"):
                raise ValueError("数据格式错误，应以CTRL:开头")

            # 去掉CTRL:前缀
            data_part = payload_str[5:]
            
            # 分割各部分数据
            parts = data_part.split(',')
            if len(parts) != 8:
                raise ValueError("数据部分应有8个值")

            # 解析摇杆数据 (前4个值是浮点数)
            axes[0] = float(parts[2])  # 右摇杆X
            axes[1] = float(parts[3])  # 右摇杆Y
            axes[2] = float(parts[1])  # 左摇杆X
            axes[3] = float(parts[0])  # 左摇杆Y

            # 解析按钮数据 (后4个值是整数)
            axes[4] = float(parts[4])  # 按钮1
            axes[5] = float(parts[5])  # 按钮2
            axes[6] = float(parts[6])  # 按钮3
            axes[7] = float(parts[7])  # 按钮4
            if axes[4] == 1.0:
                axes[4] = -1.0
            elif axes[4] == 0.0:
                axes[4] = 1.0
            if axes[5] == 2.0:
                axes[5] = -1.0
            elif axes[5] == 1.0:
                axes[5] = 0.0
            elif axes[5] == 0.0:
                axes[5] = 1.0
            if axes[7] == 0.0:
                axes[7] = 1.0
            elif axes[7] == 1.0:
                axes[7] = 0.0
            elif axes[7] == 2.0:
                axes[7] = -1.0
            axes[0] = -axes[0]
            axes[3] = -axes[3]
            # print(f"[SUCCESS] 解析成功: {axes}")

            # 发布ROS2 Joy消息
            joy_msg = Joy()
            joy_msg.header.stamp = self.get_clock().now().to_msg()
            joy_msg.header.frame_id = "bluetooth_joy"
            joy_msg.axes = axes
            self.publisher_.publish(joy_msg)
        except Exception as e:
            print(f"解析控制数据时出错: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    try:
        bridge = BluetoothJoyBridge()
        rclpy.spin(bridge)
    except Exception as e:
        bridge.get_logger().error(f"节点启动失败: {str(e)}")
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
