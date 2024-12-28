import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import subprocess
import numpy as np
import sys
import os
from rclpy.qos import QoSProfile, ReliabilityPolicy
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
    raise ValueError("Error executing command")
    print("Error executing command:", e)
    print("Stderr output:", e.stderr)
    

class RtmpStreamNode(Node):
    def __init__(self, camera_topic):
        global output
        super().__init__('rtmp_stream')
        self.camera_topic = camera_topic
        self.bridge = CvBridge()
        self.rtmp_url = 'rtmp://119.23.220.15:1935/live/0'
        self.size_str = '960x600'
        self.ffmpeg_process = None
        self.setup_ffmpeg()
        qos_policy = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        self.subscription = self.create_subscription(
            Image,
            f"/{output}/perception/camera/image/left",
            self.listener_callback,
            qos_policy
        )
        self.subscription  # Prevent unused variable warning



    def setup_ffmpeg(self):
        # Create the ffmpeg command
        global output
        command = [
            'ffmpeg', '-y', '-an', '-f', 'rawvideo', '-vcodec', 'rawvideo',
            '-pix_fmt', 'bgr24', '-s', self.size_str, '-r', '60', '-i', '-',
            '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-preset', 'ultrafast',
            '-tune', 'zerolatency',
            '-g', '2',
            '-bufsize','0',
            '-f', 'mpegts', f'srt://119.23.220.15:8890?streamid=publish:{output}'
        ]
        # Start the ffmpeg subprocess
        try:
            self.ffmpeg_process = subprocess.Popen(
                command,
                stdin=subprocess.PIPE,
                stderr=subprocess.PIPE,
                stdout=subprocess.DEVNULL  # We don't need ffmpeg's output
            )
        except Exception as e:
            self.get_logger().error(f'Failed to start ffmpeg process: {e}')
            sys.exit(1)

    def listener_callback(self, imgmsg):
        try:
            # Convert ROS 2 image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(imgmsg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        # Resize the frame to fit the desired output size
        # h, w = frame.shape[:2]
        # target_height = 400
        # if h > w:
        #     new_h = int(float(h) / w * target_height)
        #     new_w = target_height
        # else:
        #     new_w = int(float(w) / h * target_height)
        #     new_h = target_height

        # # Ensure the width is no more than 640 pixels
        # if new_w > 640:
        #     scale_factor = 640 / new_w
        #     new_w = 640
        #     new_h = int(new_h * scale_factor)

        # frame = cv2.resize(frame, (new_w, new_h))
        # frame = frame[:, :640, :]  # Crop width to 640 if necessary

        # Write the frame to ffmpeg's stdin
        # success, buffer = cv2.imencode('.png', frame)
        raw_frame_bytes = frame.tobytes()
        # if not success:
        #     self.get_logger().error('Failed to encode frame')
        #     return

        self.ffmpeg_process.stdin.write(raw_frame_bytes)
        self.ffmpeg_process.stdin.flush()

        # Optionally, display the frame using OpenCV
        # cv2.imshow('Frame', frame)
        # cv2.waitKey(1)

    def cleanup(self):
        # Terminate the ffmpeg process when the node is shut down
        if self.ffmpeg_process is not None:
            self.ffmpeg_process.stdin.close()
            self.ffmpeg_process.terminate()
            self.ffmpeg_process.wait()

def main(args=None):
    rclpy.init(args=args)
    camera_topic = '/tita3037207/perception/camera/image/left'  # Default topic
    rtmp_stream_node = RtmpStreamNode(camera_topic)

    try:
        rclpy.spin(rtmp_stream_node)
    except KeyboardInterrupt:
        pass
    finally:
        rtmp_stream_node.cleanup()
        rtmp_stream_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
