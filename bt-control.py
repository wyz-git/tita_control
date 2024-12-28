import subprocess
import threading
import time
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import cv2
from cv_bridge import CvBridge, CvBridgeError

camSet = "nvarguscamerasrc sensor_id=0 ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1200, format=(string)NV12, framerate=(fraction)60/1 ! queue ! nvvidconv ! video/x-raw,width=1920,height=1200,format=BGRx ! queue ! videoconvert ! appsink drop=True"

output_file = '/home/robot/btmon_output.txt'

axes1 = 0
axes2 = 0
axes3 = 0
axes4 = 0
axes5 = 1
axes6 = 1
axes7 = 0
axes8 = 0

axes_mode = 1
output = None
stop_time = 0
mode = 0

with open(output_file, 'w') as file:
    pass

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
    raise ValueError("Error executing command")
    print("Error executing command:", e)
    print("Stderr output:", e.stderr)

    
def read_file_in_real_time(file_path):
    # 初始化变量
    last_size = 0
    search_bytes = b'\x00\x04\x00\x1b'  # 要搜索的十六进制序列
    acl_handle_found = False  # 标记是否找到了 ACL Handle
    global axes5
    global axes3
    global axes4
    global axes6
    global axes7
    global axes8
    global axes_mode,mode
    while True:
        # 获取当前文件大小
        current_size = os.path.getsize(file_path)
        # 如果文件大小增加了
        if current_size > last_size:
            # 打开文件并读取新增的内容
            with open(file_path, 'rb') as file:
                file.seek(last_size)
                new_data = file.read(current_size - last_size)
                # 在新数据中搜索特定的十六进制序列
                search_result = new_data.find(search_bytes)
                while search_result != -1:
                    # 计算 ACL Handle 的索引
                    acl_handle_index = search_result + len(search_bytes)
                    if acl_handle_index + 4 <= len(new_data):
                        # 读取 ACL Handle
                        acl_handle = new_data[acl_handle_index:acl_handle_index + 2]
                        # 读取 ACL Handle 后紧接着的两个字节
                        acl_data = new_data[acl_handle_index + 2:acl_handle_index + 4]
                        #print(f"Found ACL Handle: {acl_handle.hex()}")
                        #print(f"ACL Data: {acl_data.hex()}")
                        if acl_handle.hex() == "1f00" and acl_data.hex() == "0050":
                            if axes_mode==1:
                                axes3 = 0.0
                                axes4 = 0.0
                        elif acl_handle.hex() == "1f00" and acl_data.hex() == "0010":
                            axes3 = -0.5
                        elif acl_handle.hex() == "1f00" and acl_data.hex() == "0090":
                            axes3 = 0.5
                        elif acl_handle.hex() == "1f00" and acl_data.hex() == "0040":
                            axes4 = 0.5
                        elif acl_handle.hex() == "1f00" and acl_data.hex() == "0060":
                            axes4 = -0.5
                        elif acl_handle.hex() == "1f00" and acl_data.hex() == "8050":
                            if axes5 == 1:
                                axes5 = -1
                            else:
                                axes5 = 1
                        elif acl_handle.hex() == "1f00" and acl_data.hex() == "1052":
                            if axes6 == -1:
                                axes6 = 1
                            else:
                                axes6 = -1
                        elif acl_handle.hex() == "1f00" and acl_data.hex() == "4050":
                            mode = 4
                            if axes_mode == 1:
                                axes_mode = 0
                            else:
                                axes_mode = 1
                                axes3=0
                                axes4 = 0
                        elif acl_handle.hex() == "1f00" and acl_data.hex() == "0851":
                            if axes7 == 0:
                                axes7 = 1
                            else:
                                axes7 = 0
                        elif acl_handle.hex() == "1f00" and acl_data.hex() == "0150":
                            if axes8 == 0:
                                axes8 = 1
                            elif axes8 == 1:
                                axes8 = -1
                            else:
                                axes8 = 0
                        acl_handle_found = True  # 标记为已找到
                    else:
                        # 数据不足，等待下一次读取
                        print("New data is not enough to contain ACL Handle and ACL Data after the sequence.")

                    # 移动搜索起始点，继续搜索下一个可能的匹配项（如果有的话）
                    search_result = new_data.find(search_bytes, search_result + 1)

            # 更新 last_size 以反映当前文件大小
            last_size = current_size

        # 如果文件大小减小（虽然这种情况很少见，但理论上可能发生），更新 last_size
        if current_size < last_size:
            last_size = current_size

        # 如果已经找到了 ACL Handle 并且您想停止循环，可以在这里添加逻辑
        # 例如：if acl_handle_found: break

        # 等待一段时间再检查
        time.sleep(0.1)

class JoyPublisher(Node):
    def __init__(self):
        global output
        super().__init__('joy_publisher_bt')
        self.publisher_ = self.create_publisher(Joy, f"/{output}/joy", 10)  # 创建一个发布者
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.bridge = CvBridge()
        self.stop_time = 0

    def timer_callback(self):
        global axes5
        global axes3
        global axes4
        global axes6
        global axes7
        global axes8
        global axes_mode,mode

        if mode == 4:
            if self.stop_time < 10:
                self.stop_time = self.stop_time + 1
            elif self.stop_time >= 10:    
                mode = 0
        else:
            self.stop_time = 0
        if axes_mode == 0 and mode == 0:
            return
        
        joy_msg = Joy()
        joy_msg.header.frame_id = "joy"
        joy_msg.axes = [
            float(0),
            float(0),
            float(axes3),
            float(axes4),
            float(axes5),
            float(axes6),
            float(axes7),
            float(axes8),
            float(mode)
        ]
        self.publisher_.publish(joy_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyPublisher()

    process = subprocess.Popen(["sudo",'btmon', '-w', output_file], stderr=subprocess.PIPE)
    reader_thread = threading.Thread(target=read_file_in_real_time, args=(output_file,))
    reader_thread.start()
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
