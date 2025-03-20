import json
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt

class MqttToRos2Bridge(Node):
    def __init__(self, robot_name):
        super().__init__('mqtt_to_ros2_bridge')

        self.robot_name = robot_name

        # 创建 ROS2 Publisher
        self.publisher = self.create_publisher(String, f'/{self.robot_name}/switch_states', 10)

        # MQTT 客户端配置
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message

        # 连接到 MQTT 服务器
        self.mqtt_client.connect("119.23.220.15", 1883, 60)

    def on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT 连接成功回调函数"""
        self.get_logger().info("Connected to MQTT broker with result code: " + str(rc))
        # 订阅 MQTT Topic
        client.subscribe(f"{self.robot_name}-control")  # 动态订阅 MQTT Topic

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

def get_robot_name():
    """通过执行命令获取机器人名称"""
    try:
        # 执行 ros2 topic list 命令
        ros2_topic_list = subprocess.run(
            ['ros2', 'topic', 'list'],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True
        )

        # 使用 grep 过滤包含 'tita' 的行
        grep_tita = subprocess.run(
            ['grep', 'tita'],
            input=ros2_topic_list.stdout, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True
        )

        # 使用 awk 提取第一个部分
        awk_extract = subprocess.run(
            ['awk', '-F/', '{print $2}'],
            input=grep_tita.stdout, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True
        )

        # 使用 head 获取第一行
        head_first_line = subprocess.run(
            ['head', '-n', '1'],
            input=awk_extract.stdout, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True
        )

        # 使用 sed 去除多余部分
        sed_clean = subprocess.run(
            ['sed', 's/\\..*//'],
            input=head_first_line.stdout, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True
        )

        # 获取最终结果
        robot_name = sed_clean.stdout.strip()
        if not robot_name.startswith('tita'):
            raise ValueError(f"Extracted topic name does not start with 'tita'. Extracted name: {robot_name}")
        return robot_name
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"Failed to execute command: {e}")

def main(args=None):
    # 初始化 ROS2
    rclpy.init(args=args)

    # 获取机器人名称
    try:
        robot_name = get_robot_name()
        print(f"Extracted robot name: {robot_name}")
    except Exception as e:
        print(f"Error: {e}")
        return

    # 创建 MQTT 到 ROS2 的桥接节点
    bridge = MqttToRos2Bridge(robot_name)

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