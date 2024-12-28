import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import time
import paho.mqtt.client as mqtt
import subprocess

elrs_publisher = None
node = None
output = None

# 定义要执行的命令，注意这里使用了列表形式来避免shell注入风险
command = [
    'ros2', 'topic', 'list',
    '|', 'grep', 'tita',
    '|', 'awk', '-F/', "'{print $2}'",
    '|', 'head', '-n', '1',
    '|', 'sed', "'s/\\..*//'"
]

# 由于管道操作符'|'在shell中用于连接多个命令，但在Python的subprocess中不能直接使用
# 我们需要将命令拆分成多个部分，并逐个执行，通过管道连接它们的输出和输入
# 但为了简化，这里我们采用一种方法，将整个命令作为一个字符串传递给shell（注意风险）
# 如果要更安全地执行，需要分别调用每个命令并处理它们的输出
# 但由于您的命令相对简单且固定，这里采用简化方法
# 注意：这种方法存在shell注入风险，如果命令中的输入（如'tita'）来自不可信的源，则不应使用
command_str = ' '.join(command)

try:
    # 使用subprocess.run来执行命令并捕获输出
    result = subprocess.run(command_str, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    
    # 获取命令的输出（stdout）
    output = result.stdout.strip()
    # 检查输出是否以'tita'开头（这里假设大小写敏感）
    if not output.startswith('tita'):
        # 输出错误消息或引发异常
        print("Error: Extracted topic name does not start with 'tita'. Extracted name:", output)
        raise ValueError("Extracted topic name does not start with 'tita'.")
    else:
        print("Extracted topic name:", output)
    
except subprocess.CalledProcessError as e:
    # 如果命令执行失败（返回非零退出码），则捕获异常并打印错误信息
    print("Error executing command:", e)
    print("Stderr output:", e.stderr)



# MQTT回调函数
def on_connect(client, userdata, flags, rc):
    global output
    """连接MQTT服务器后的回调函数"""
    print(f"Connected with result code {str(rc)}")
    # 订阅主题
    client.subscribe(output)

def crsf_crc8(ptr, start, length):
    crsf_crc8tab = [
        0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
        0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
        0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
        0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
        0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
        0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
        0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
        0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
        0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
        0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
        0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
        0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
        0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
        0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
        0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
        0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
    ]
    crc = 0
    for i in range(start, start + length):
        crc = crsf_crc8tab[crc ^ ptr[i]]
    return crc

def readPacket(inData, len):
    frameLength = 0
    bufferIndex = 0
    m_inBuffer = [0] * 26
    if len >= 1 and inData[0] == 0xC8:  # Assuming CRSF_ADDRESS_FLIGHT_CONTROLLER is 0x5E
        m_inBuffer[bufferIndex] = inData[0]
        bufferIndex += 1
        if len >= 2:
            frameLength = inData[1]
            m_inBuffer[bufferIndex] = inData[1]
            bufferIndex += 1
    if bufferIndex > 1 and bufferIndex <= frameLength:
        for i in range(bufferIndex, frameLength + 1):
            if i < len:
                m_inBuffer[i] = inData[i]
                bufferIndex += 1
    if bufferIndex == frameLength + 1 and len >= 25:
        m_inBuffer[bufferIndex] = inData[25]  # Assuming CRC is at position 24
        crc = crsf_crc8(m_inBuffer, 2, frameLength - 1)  # Calculate CRC excluding address and CRC byte

        if frameLength == 24 and m_inBuffer[0] == 0xC8 and crc == inData[25]:
            m_channels = [0] * 16
            if m_inBuffer[1] == 24:
                m_channels[0] = ((m_inBuffer[3] | m_inBuffer[4] << 8) & 0x07FF)
                m_channels[1] = ((m_inBuffer[4] >> 3 | m_inBuffer[5] << 5) & 0x07FF)
                m_channels[2] = ((m_inBuffer[5] >> 6 | m_inBuffer[6] << 2 | m_inBuffer[7] << 10) & 0x07FF)
                m_channels[3] = ((m_inBuffer[7] >> 1 | m_inBuffer[8] << 7) & 0x07FF)
                m_channels[4] = ((m_inBuffer[8] >> 4 | m_inBuffer[9] << 4) & 0x07FF)
                m_channels[5] = ((m_inBuffer[9] >> 7 | m_inBuffer[10] << 1 | m_inBuffer[11] << 9) & 0x07FF)
                m_channels[6] = ((m_inBuffer[11] >> 2 | m_inBuffer[12] << 6) & 0x07FF)
                m_channels[7] = ((m_inBuffer[12] >> 5 | m_inBuffer[13] << 3) & 0x07FF)
                m_channels[8] = ((m_inBuffer[14] | m_inBuffer[15] << 8) & 0x07FF)
                m_channels[9] = ((m_inBuffer[15] >> 3 | m_inBuffer[16] << 5) & 0x07FF)
                m_channels[10] = ((m_inBuffer[16] >> 6 | m_inBuffer[17] << 2 | m_inBuffer[18] << 10) & 0x07FF)
                m_channels[11] = ((m_inBuffer[18] >> 1 | m_inBuffer[19] << 7) & 0x07FF)
                m_channels[12] = ((m_inBuffer[19] >> 4 | m_inBuffer[20] << 4) & 0x07FF)
                m_channels[13] = ((m_inBuffer[20] >> 7 | m_inBuffer[21] << 1 | m_inBuffer[22] << 9) & 0x07FF)
                m_channels[14] = ((m_inBuffer[22] >> 2 | m_inBuffer[23] << 6) & 0x07FF)
                m_channels[15] = ((m_inBuffer[23] >> 5 | m_inBuffer[24] << 3) & 0x07FF)
                m_channels_public(m_channels)

def normalize_value(original_value, min_original, range_original):
    return ((original_value - min_original) / range_original) * 2 - 1


def m_channels_public(m_channels):
    # 创建一个Joy消息
    # global node
    global elrs_publisher
    joy_msg = Joy()
    # joy_msg.header.stamp = node.get_clock().now()
    joy_msg.header.frame_id = "joy"

    # 归一化前四个通道
    normalized1 = -1.0 * normalize_value(m_channels[0], 172, 1638)
    normalized2 = -1.0 * normalize_value(m_channels[1], 172, 1638)
    normalized3 = -1.0 * normalize_value(m_channels[2], 172, 1638)
    normalized4 = -1.0 * normalize_value(m_channels[3], 172, 1638)

    # 应用偏差阈值
    if abs(normalized1) < 0.0013:
        normalized1 = 0.0
    if abs(normalized2) < 0.0013:
        normalized2 = 0.0
    if abs(normalized3) < 0.0013:
        normalized3 = 0.0
    if abs(normalized4) < 0.0013:
        normalized4 = 0.0

    # 填充axes数组
    joy_msg.axes = [
        float(normalized1),
        float(normalized2),
        float(normalized3),
        float(normalized4),
        float(-(m_channels[4] // 800 - 1)),  # 注意这里可能仍然不是你想要的，因为//是整数除法
        float(-(m_channels[5] // 900 - 1)),
        float(-(m_channels[6] // 800 - 1)),
        float(-(m_channels[7] // 900 - 1)),
        float((m_channels[8] - 988) / 4)  # 使用/而不是//来进行浮点除法
    ]
    elrs_publisher.publish(joy_msg)

def on_message(client, userdata, msg):
    readPacket(msg.payload,len(msg.payload))

def main(args=None):
    rclpy.init(args=args)

    global node,output,elrs_publisher

    node = Node("joy_mqtt_publisher")

    elrs_publisher = node.create_publisher(Joy, f"/{output}/joy", 10)

    # 初始化MQTT客户端
    client = mqtt.Client()

    # 绑定回调函数
    client.on_connect = on_connect
    client.on_message = on_message

    # 连接到MQTT服务器
    client.connect("119.23.220.15", 1883, 10)

    try:
        client.loop_forever()

    except KeyboardInterrupt:
        print("程序被手动中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

