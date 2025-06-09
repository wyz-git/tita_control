#!/usr/bin/env python3
import re
import rclpy
import json
import os
from pathlib import Path
from rclpy.node import Node
from sensor_msgs.msg import Joy
import subprocess
import paho.mqtt.client as mqtt

class MqttJoyAxesBridge(Node):
    def __init__(self):
        super().__init__('mqtt_joy_axes_bridge')
        
        # 加载配置文件
        self.config = self.load_config()
        
        # ROS2 配置
        self.publisher_ = self.create_publisher(Joy, f"/{self.config['mqtt_username']}/joy", 10)
        self.mqtt_topic = f"{self.config['mqtt_username']}/virtual"

        
        # MQTT 配置
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.username_pw_set(
            self.config["mqtt_username"],
            self.config["mqtt_password"]
        )
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        # 初始化 axes 数组
        self.axes = [0.0] * 8
        
        # 连接MQTT
        self.mqtt_client.connect("119.23.220.15", 1883, 60)
        self.mqtt_client.loop_start()

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

    def on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"MQTT Connected with code {rc}")
        client.subscribe(self.mqtt_topic)
        
    def on_mqtt_message(self, client, userdata, msg):
        try:
            # 原始字节数据转ASCII字符串
            payload = msg.payload.decode('ascii', errors='ignore').strip()
            # print(f"\n[DEBUG] 原始数据: {payload}")

            # 初始化数据结构
            axes = [0.0] * 8
            parsed_buttons = False

            ##################################################
            # 步骤一：解析摇杆数据
            ##################################################
            # 寻找左摇杆起始位置 L[
            l_start = payload.find("L[")
            if l_start == -1:
                raise ValueError("左摇杆数据格式错误")

            # 提取左摇杆数值
            l_values, r_start = self._parse_joystick_values(payload, l_start + 2)
            axes[3], axes[2] = l_values

            # 寻找右摇杆起始位置 R[
            r_start = payload.find("R[", r_start)
            if r_start == -1:
                raise ValueError("右摇杆数据格式错误")

            # 提取右摇杆数值
            r_values, btn_start = self._parse_joystick_values(payload, r_start + 2)
            axes[0], axes[1] = r_values

            ##################################################
            # 步骤二：解析按钮数据
            ##################################################
            # 寻找按钮起始位置 BTN: [
            btn_start = payload.find("BTN: [", btn_start)
            if btn_start == -1:
                raise ValueError("按钮数据格式错误")

            # 提取按钮数值
            btn_values = self._parse_button_values(payload, btn_start + 6)
            axes[4:8] = [float(v) for v in btn_values]

            ##################################################
            # 发布ROS2消息
            ##################################################
            if axes[4] == 1.0:
                axes[4] = -1.0
            elif axes[4] == 0.0:
                axes[4] = 1.0
            if axes[5] == 2.0:
                axes[5] = -1.0
            if axes[7] == 0.0:
                axes[7] = 1.0
            elif axes[7] == 1.0:
                axes[7] = 0.0
            elif axes[7] == 2.0:
                axes[7] = -1.0
            axes[0] = -axes[0]
            axes[3] = -axes[3]
            joy_msg = Joy()
            joy_msg.header.frame_id = "joy"
            joy_msg.axes = axes
            self.publisher_.publish(joy_msg)
            # print(f"[SUCCESS] 解析成功: {axes}")

        except Exception as e:
            print(f"[ERROR] 解析失败: {str(e)}")

    def _parse_joystick_values(self, payload, start_pos):
        """解析摇杆数值（返回两个浮点数及结束位置）"""
        values = []
        current_num = []
        pos = start_pos

        while pos < len(payload):
            c = payload[pos]
            if c == ']':
                # 结束标记
                if current_num:
                    values.append(float(''.join(current_num)))
                if len(values) != 2:
                    raise ValueError("摇杆数值数量错误")
                return (values[0], values[1]), pos + 1
            elif c == ',':
                # 数值分隔符
                if current_num:
                    values.append(float(''.join(current_num)))
                    current_num = []
                pos += 1
                # 跳过逗号后的空格
                while pos < len(payload) and payload[pos].isspace():
                    pos += 1
                continue
            elif c in ('-', '+') or c.isdigit() or c == '.':
                # 数值字符
                current_num.append(c)
            elif c.isspace():
                # 忽略空格
                pass
            else:
                # 非法字符
                raise ValueError(f"非法字符 '{c}' (ASCII: {ord(c)})")
            pos += 1

        raise ValueError("摇杆数据未闭合")

    def _parse_button_values(self, payload, start_pos):
        """解析按钮数值（返回四个整数）"""
        values = []
        current_num = []
        pos = start_pos

        while pos < len(payload):
            c = payload[pos]
            if c == ']':
                # 结束标记
                if current_num:
                    values.append(int(''.join(current_num)))
                if len(values) != 4:
                    raise ValueError("按钮数值数量错误")
                return values
            elif c == ',':
                # 数值分隔符
                if current_num:
                    values.append(int(''.join(current_num)))
                    current_num = []
                pos += 1
                # 跳过逗号后的空格
                while pos < len(payload) and payload[pos].isspace():
                    pos += 1
                continue
            elif c.isdigit():
                # 数字字符
                current_num.append(c)
            elif c.isspace():
                # 忽略空格
                pass
            else:
                # 非法字符
                raise ValueError(f"非法字符 '{c}' (ASCII: {ord(c)})")
            pos += 1

        raise ValueError("按钮数据未闭合")
    
def main(args=None):
    rclpy.init(args=args)
    try:
        bridge = MqttJoyAxesBridge()
        rclpy.spin(bridge)
    except Exception as e:
        bridge.get_logger().error(f"节点启动失败: {str(e)}")
    finally:
        bridge.mqtt_client.loop_stop()
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
