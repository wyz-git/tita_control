import json
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt
from pathlib import Path

class MqttToRos2Bridge(Node):
    def __init__(self):
        super().__init__('mqtt_to_ros2_bridge')

        # 加载配置文件
        self.config = self.load_config()
        
        # ROS2 配置
        self.publisher = self.create_publisher(String, f"/{self.config['mqtt_username']}/switch_states", 10)
        self.mqtt_topic = f"{self.config['mqtt_username']}/switches"

        # MQTT 配置
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.username_pw_set(
            self.config["mqtt_username"],
            self.config["mqtt_password"]
        )
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        
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
        """MQTT 消息接收回调函数"""
        #self.get_logger().info(f"Received MQTT message: {msg.payload.decode()}")

        # 解析 JSON 消息
        try:
            switch_states = json.loads(msg.payload.decode())
            #self.get_logger().info(f"Parsed switch states: {switch_states}")

            # 发布到 ROS2 Topic
            ros2_msg = String()
            ros2_msg.data = json.dumps(switch_states)  # 将 JSON 转换为字符串
            self.publisher.publish(ros2_msg)
            #self.get_logger().info(f"Published to ROS2 topic: /{self.robot_name}/switch_states")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse MQTT message: {e}")

def main(args=None):
    # 初始化 ROS2
    rclpy.init(args=args)

    # 创建 MQTT 到 ROS2 的桥接节点
    bridge = MqttToRos2Bridge()

    # 启动 MQTT 客户端循环
    bridge.mqtt_client.loop_start()

    # 保持 ROS2 节点运行
    rclpy.spin(bridge)

    # 关闭 MQTT 客户端
    bridge.mqtt_client.loop_stop()
    bridge.mqtt_client.disconnect()

    # 关闭 ROS2 节点
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
