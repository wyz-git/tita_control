import serial  
import paho.mqtt.client as mqtt  
import time  
  
# MQTT配置  
MQTT_SERVER = "119.23.220.15"  
MQTT_PORT = 1883  
MQTT_TOPIC = "tita3037207"  
MQTT_CLIENT_ID = "serial_to_mqtt"  
  
# 串口配置  linux:/dev/ttyUSB* windows:COM5
SERIAL_PORT = 'COM5'  
SERIAL_BAUDRATE = 420000 
  
# MQTT回调函数  
def on_connect(client, userdata, flags, rc):  
    print("MQTT Connected with result code "+str(rc))  
  
    # 订阅主题（可选，如果你需要订阅消息）  
    # client.subscribe("another/topic")  
  
def on_message(client, userdata, msg):  
    print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")  
  
# MQTT客户端  
client = mqtt.Client(MQTT_CLIENT_ID)  
client.on_connect = on_connect  
client.on_message = on_message  
  
# 连接到MQTT服务器  
client.connect(MQTT_SERVER, MQTT_PORT, 60)  
  
# 保持MQTT连接  
client.loop_start()  
  
# 串口配置  
ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=1)  

# 用于累积串口数据的缓冲区
serial_buffer = bytearray()

try:
    while True:
        # 读取串口数据
        if ser.in_waiting:
            new_data = ser.read(ser.in_waiting)
            serial_buffer.extend(new_data)

        # 查找并处理所有以 0xc8 开头的消息
        index = 0  # 用于遍历缓冲区的索引
        while index < len(serial_buffer):
            if serial_buffer[index] == 0xc8 and len(serial_buffer) - index >= 26:
                # 找到了一个有效的消息
                message_to_send = serial_buffer[index:index+26]
                client.publish(MQTT_TOPIC, message_to_send)
                # 更新索引以跳过已处理的消息
                index += 26
                serial_buffer = serial_buffer[index:]
                index = 0  # 如果执行了上面的切片，则需要重置index
            else:
                # 没有找到有效的消息，继续检查下一个字节
                index += 1

#try:  
 #   while True:  
  #      if ser.in_waiting:
            #data = ser.readline().strip().decode('utf-8', 'ignore')
   #         data = ser.readline().strip() 
            #print(f"Received from serial: {data}")  
    #        client.publish(MQTT_TOPIC, data)  # 发布到MQTT  
       # time.sleep(0.05)  # 稍作延时，避免过于频繁读取  
except KeyboardInterrupt:  
    print("Program stopped by user.")  
finally:  
    ser.close()  # 关闭串口  
    client.loop_stop()  # 停止MQTT循环  
    client.disconnect()  # 断开MQTT连接
