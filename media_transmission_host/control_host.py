import serial
import paho.mqtt.client as mqtt
import time
from typing import Optional

class SerialToMqttBridge:
    def __init__(self):
        """初始化桥接器"""
        # 配置参数
        self.mqtt_config = {
            "server": "119.23.220.15",
            "port": 1883,
            "topic": "ao2car2063135",
            "client_id": "serial_to_mqtt"
        }

        self.serial_config = {
            "port": 'COM8',
            "baudrate": 420000,
            "timeout": 1
        }

        # 运行状态
        self.serial_buffer = bytearray()
        self.running = True
        self.ser: Optional[serial.Serial] = None
        self.mqtt_client: Optional[mqtt.Client] = None

        # 初始化硬件连接
        self._init_connections()

    def _init_connections(self):
        """初始化硬件连接"""
        try:
            # 初始化串口
            self.ser = serial.Serial(
                port=self.serial_config["port"],
                baudrate=self.serial_config["baudrate"],
                timeout=self.serial_config["timeout"]
            )

            # 初始化MQTT客户端（V1 API）
            self.mqtt_client = mqtt.Client(client_id=self.mqtt_config["client_id"])
            self.mqtt_client.on_connect = self._on_mqtt_connect
            self.mqtt_client.on_message = self._on_mqtt_message
            self.mqtt_client.connect(
                host=self.mqtt_config["server"],
                port=self.mqtt_config["port"],
                keepalive=60
            )
            self.mqtt_client.loop_start()

        except serial.SerialException as se:
            print(f"串口初始化失败: {str(se)}")
            self.running = False
        except Exception as e:
            print(f"初始化异常: {str(e)}")
            self.running = False

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT连接回调（V1 API）"""
        print(f"MQTT连接成功，返回码: {rc}")
        if rc == 0:
            print("已连接到MQTT服务器")
        else:
            print(f"连接失败，错误码: {rc}")

    def _on_mqtt_message(self, client, userdata, msg):
        """MQTT消息回调（基础功能）"""
        try:
            payload = msg.payload.decode('utf-8')
            print(f"收到MQTT消息: [{msg.topic}] {payload}")
        except UnicodeDecodeError:
            print(f"收到二进制消息，长度: {len(msg.payload)}字节")

    def _process_serial_data(self):
        """处理串口数据"""
        try:
            index = 0
            while index < len(self.serial_buffer):
                # 查找有效消息头
                if self.serial_buffer[index] == 0xC8 and (len(self.serial_buffer) - index) >= 26:
                    self._handle_valid_packet(index)
                    index = 0  # 处理完成后重置索引
                else:
                    index += 1
        except Exception as e:
            print(f"数据处理异常: {str(e)}")

    def _handle_valid_packet(self, index: int):
        """处理有效数据包"""
        packet = self.serial_buffer[index:index+26]
        try:
            if self.mqtt_client:
                self.mqtt_client.publish(self.mqtt_config["topic"], bytes(packet))
            del self.serial_buffer[:index+26]
        except BrokenPipeError:
            print("MQTT连接中断，尝试重连...")
            self._reconnect_mqtt()

    def _reconnect_mqtt(self):
        """MQTT重连机制"""
        try:
            if self.mqtt_client:
                self.mqtt_client.reconnect()
                print("MQTT重连成功")
        except Exception as e:
            print(f"重连失败: {str(e)}")

    def run(self):
        """主运行循环"""
        try:
            while self.running and self.ser and self.ser.is_open:
                # 读取串口数据
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    self.serial_buffer.extend(data)

                # 处理数据
                self._process_serial_data()

                # 降低CPU占用
                time.sleep(0.01)

        except KeyboardInterrupt:
            print("用户中断操作")
        except Exception as e:
            print(f"运行时异常: {str(e)}")
        finally:
            self._cleanup()

    def _cleanup(self):
        """资源清理"""
        print("正在清理资源...")
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception as e:
            print(f"关闭串口失败: {str(e)}")

        try:
            if self.mqtt_client:
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
        except Exception as e:
            print(f"断开MQTT失败: {str(e)}")

if __name__ == "__main__":
    bridge = SerialToMqttBridge()
    bridge.run()