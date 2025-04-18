import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import paho.mqtt.client as mqtt
import subprocess
from std_msgs.msg import String
import json
import threading
import os 

class JoyMQTTNode(Node):
    def __init__(self):
        super().__init__("joy_mqtt_publisher")
        
        # 获取MQTT主题名称
        self.output = self._get_topic_name()
        self.get_logger().info(f"Using topic prefix: {self.output}")
        
        # 初始化ROS2发布者
        self.elrs_publisher = self.create_publisher(Joy, f"/{self.output}/joy", 10)
        self.subscription_switch = self.create_subscription(
            String,
            f"/{self.output}/switch_states",
            self.switch_states_callback,
            10
        )
        # 配置MQTT客户端
        self.axes_mode = True
        self._init_mqtt_client()

    def _on_mqtt_disconnect(self, client, userdata, rc):

        self.get_logger().error(f"连接断开")
        self.destroy_node()
        os._exit(1)

    def _init_mqtt_client(self):
        """初始化并启动MQTT客户端线程"""
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_message = self._on_mqtt_message
        self.mqtt_client.connect("119.23.220.15", 1883, 10)
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
        # 在独立线程中运行MQTT循环
        self.mqtt_thread = threading.Thread(target=self._mqtt_loop)
        self.mqtt_thread.daemon = True
        self.mqtt_thread.start()

    def switch_states_callback(self, msg):
        """订阅 /ao2car3037207/switch_states 的回调函数"""
        try:
            # 解析 JSON 消息
            switch_states = json.loads(msg.data)
            self.axes_mode = switch_states.get("8", 0)  # 如果不存在 "4"，默认值为 0
            # self.get_logger().info(f"Updated axes_mode: {self.axes_mode}")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse switch_states message: {e}")

    def _get_topic_name(self):
        robot_name = os.environ.get('ROBOT_NAME')
        if robot_name:
            self.get_logger().info(f"Using ROBOT_NAME from env: {robot_name}")
            return robot_name

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT连接回调"""
        self.get_logger().info(f"MQTT connected with code {rc}")
        client.subscribe(self.output)

    def _on_mqtt_message(self, client, userdata, msg):
        """MQTT消息回调"""
        self._process_packet(msg.payload, len(msg.payload))

    def _process_packet(self, in_data, length):
        """处理CRSF数据包"""
        # 此处保留原始数据解析逻辑
        if length < 26 or in_data[0] != 0xC8:
            return

        frame_length = in_data[1]
        if frame_length != 24 or not self._validate_crc(in_data):
            return

        channels = self._decode_channels(in_data)
        self._publish_joy(channels)

    def _validate_crc(self, data):
        """校验CRC"""
        crc = self._crsf_crc8(data[2:25])
        return crc == data[25]

    def _crsf_crc8(self, data):
        """计算CRC8"""
        crsf_crc8tab = [
            0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 
            0x83, 0xD7, 0x02, 0xA8, 0x7D, 0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 
            0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F, 0xA4, 
            0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 
            0x73, 0xA6, 0x0C, 0xD9, 0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 
            0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B, 0x9D, 0x48, 
            0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 
            0x9F, 0x35, 0xE0, 0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 
            0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2, 0x39, 0xEC, 0x46, 
            0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 
            0x91, 0x44, 0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 
            0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16, 0xEF, 0x3A, 0x90, 0x45, 
            0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 
            0x92, 0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 
            0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0, 0x4B, 0x9E, 0x34, 0xE1, 0xB5, 
            0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36, 
            0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 
            0x9A, 0xCE, 0x1B, 0xB1, 0x64, 0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 
            0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F, 0x20, 
            0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 
            0xF7, 0x22, 0x88, 0x5D, 0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 
            0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB, 0x84, 0x51, 
            0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 
            0x86, 0x2C, 0xF9
        ]
        crc = 0
        for byte in data:
            crc = crsf_crc8tab[crc ^ byte]
        return crc

    def _decode_channels(self, data):
        """解码16个通道数据"""
        return [
            ((data[3] | data[4] << 8) & 0x07FF),
            ((data[4] >> 3 | data[5] << 5) & 0x07FF),
            ((data[5] >> 6 | data[6] << 2 | data[7] << 10) & 0x07FF),
            ((data[7] >> 1 | data[8] << 7) & 0x07FF),
            ((data[8] >> 4 | data[9] << 4) & 0x07FF),
            ((data[9] >> 7 | data[10] << 1 | data[11] << 9) & 0x07FF),
            ((data[11] >> 2 | data[12] << 6) & 0x07FF),
            ((data[12] >> 5 | data[13] << 3) & 0x07FF),
            ((data[14] | data[15] << 8) & 0x07FF),
            ((data[15] >> 3 | data[16] << 5) & 0x07FF),
            ((data[16] >> 6 | data[17] << 2 | data[18] << 10) & 0x07FF),
            ((data[18] >> 1 | data[19] << 7) & 0x07FF),
            ((data[19] >> 4 | data[20] << 4) & 0x07FF),
            ((data[20] >> 7 | data[21] << 1 | data[22] << 9) & 0x07FF),
            ((data[22] >> 2 | data[23] << 6) & 0x07FF),
            ((data[23] >> 5 | data[24] << 3) & 0x07FF)
        ]

    def _publish_joy(self, channels):
        """发布Joy消息"""
        if self.axes_mode == True:
            def _normalize(value):
                return ((value - 172) / 1638) * 2 - 1

            msg = Joy()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "elrs_joy"
            
            # 处理前四个模拟通道
            axes = [
                -_normalize(channels[0]),
                -_normalize(channels[1]),
                -_normalize(channels[2]),
                -_normalize(channels[3]),
                float(-(channels[4] // 800 - 1)),
                float(-(channels[5] // 900 - 1)),
                float(-(channels[6] // 800 - 1)),
                float(-(channels[7] // 900 - 1)),
                (channels[8] - 988) / 4
            ]
            # 应用死区过滤
            for i in range(4):
                if abs(axes[i]) < 0.0013:
                    axes[i] = 0.0
            
            msg.axes = axes
            self.elrs_publisher.publish(msg)

    def _mqtt_loop(self):
        """启动MQTT客户端"""
        self.mqtt_client.loop_forever()

def main(args=None):
    rclpy.init(args=args)
    node = JoyMQTTNode()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Initialization failed: {str(e)}")
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.mqtt_client.loop_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()