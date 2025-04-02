import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import time
import os
import subprocess
import select
import queue
from threading import Thread

class RollingQueue(queue.Queue):
    """自动淘汰旧元素的线程安全队列"""
    def put(self, item, block=True, timeout=None):
        while True:
            try:
                return super().put(item, block=False)
            except queue.Full:
                try:
                    self.get_nowait()
                except queue.Empty:
                    pass

class RealsensePublisher(Node):
    def __init__(self):
        super().__init__('realsense_publisher')
        if not self._check_network_reachable('119.23.220.15'):
            raise RuntimeError("网络不可达，无法连接推流服务器")
        self.output = self._get_topic_name()
        # ROS2发布器初始化
        self.image_pub = self.create_publisher(Image, f"/{self.output}/image_raw", 10)
        self.info_pub = self.create_publisher(CameraInfo, f"/{self.output}/camera_info", 10)
        self.bridge = CvBridge()

        # 推流状态监控
        self.stream_queue = RollingQueue(maxsize=60)
        self.streaming_active = True
        self.dropped_frames = 0      # 丢弃帧计数器
        self.restart_count = 0       # 重启计数器
        self.frame_counter = 0       # 成功帧计数器
        self.last_log_time = time.time()

        # 初始化推流
        self._init_ffmpeg_stream()

        # RealSense初始化
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self._init_realsense()

    def _check_network_reachable(self, host):
            if self._ping_host(host):
                return True
            return False

    def _ping_host(self, host):
        """ICMP ping检测"""
        try:
            result = subprocess.run(
                ['ping', '-c', '3', '-W', '2', host],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=10
            )
            if result.returncode == 0:
                self.get_logger().info(f"Ping {host} 成功")
                return True
        except subprocess.TimeoutExpired:
            self.get_logger().warning(f"Ping {host} 超时")
        except Exception as e:
            self.get_logger().error(f"Ping检测异常: {str(e)}")
        return False

    def _get_topic_name(self):
        robot_name = os.environ.get('ROBOT_NAME')
        if robot_name:
            self.get_logger().info(f"Using ROBOT_NAME from env: {robot_name}")
            return robot_name

    def _init_ffmpeg_stream(self):
        """初始化FFmpeg推流管道"""
        # 优化硬件加速参数
        self.ffmpeg_cmd = [
            'ffmpeg',
            '-re',
            '-f', 'rawvideo',
            '-vcodec', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-s', '848x480',
            '-r', '60',
            '-i', '-',
            '-c:v', 'libx264',
            '-vf', 'format=yuv420p',
            '-preset', 'ultrafast',
            '-tune', 'zerolatency',
            '-x264-params', 'keyint=30:min-keyint=30:scenecut=0',
            '-b:v', '3M',
            '-maxrate', '3M',
            '-bufsize', '1M',
            '-threads', '6',
            '-f', 'mpegts',
            f'srt://119.23.220.15:8890?streamid=publish:{self.output}'
        ]

        # 启动FFmpeg进程
        self.ffmpeg_proc = subprocess.Popen(
            self.ffmpeg_cmd,
            stdin=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

        # 启动监控线程
        self.writer_thread = Thread(target=self._stream_writer)
        self.writer_thread.start()
        self.stderr_thread = Thread(target=self._monitor_stderr)
        self.stderr_thread.start()

    def _monitor_stderr(self):
        """实时监控FFmpeg错误输出"""
        while self.streaming_active:
            try:
                line = self.ffmpeg_proc.stderr.readline()
                if line:
                    self.get_logger().error(
                        f"[FFmpeg] {line.decode().strip()}",
                        throttle_duration_sec=1
                    )
            except Exception as e:
                self.get_logger().error(f"stderr监控异常: {str(e)}")
                break

    def _stream_writer(self):
        """专用写线程处理队列数据"""
        last_warn_time = 0
        while self.streaming_active or not self.stream_queue.empty():
            try:
                # 队列状态监控
                # qsize = self.stream_queue.qsize()
                # if time.time() - last_warn_time > 5:
                #     # self.get_logger().info(
                #     #     f"推流队列状态: size={qsize}/30",
                #     #     throttle_duration_sec=5
                #     # )
                #     last_warn_time = time.time()

                # 获取帧数据
                # start_time = time.time()
                frame = self.stream_queue.get(timeout=1.0)
                # get_duration = time.time() - start_time

                # # 获取延迟警告
                # if get_duration > 0.1:
                #     self.get_logger().warning(
                #         f"取帧耗时过长: {get_duration:.3f}s"
                #     )

                # 写入处理
                self._safe_write_frame(frame)
                self.stream_queue.task_done()
                self.frame_counter += 1

                # 吞吐量统计
                # if time.time() - self.last_log_time > 1.0:
                #     self.get_logger().info(
                #         f"推流吞吐量: {self.frame_counter}fps",
                #         throttle_duration_sec=1
                #     )
                #     self.frame_counter = 0
                #     self.last_log_time = time.time()

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"写线程异常: {str(e)}")
                self._restart_ffmpeg()

    def _safe_write_frame(self, frame):
        """安全写入帧数据"""
        try:
            # 进程状态检查
            if self.ffmpeg_proc.poll() is not None:
                self.get_logger().error("FFmpeg进程异常退出！")
                self._restart_ffmpeg()
                return

            # 管道状态检查
            if self.ffmpeg_proc.stdin.closed:
                self.get_logger().error("管道已关闭")
                return

            # 带超时的写入检测
            start_time = time.time()
            _, writable, _ = select.select([], [self.ffmpeg_proc.stdin], [], 0.5)
            select_duration = time.time() - start_time

            if select_duration > 0.3:
                self.get_logger().warning(
                    f"select阻塞: {select_duration:.3f}s"
                )

            if writable:
                write_start = time.time()
                self.ffmpeg_proc.stdin.write(frame.tobytes())
                self.ffmpeg_proc.stdin.flush()
                write_duration = time.time() - write_start

                # if write_duration > 0.1:
                #     self.get_logger().warning(
                #         f"大帧写入耗时: {write_duration:.3f}s 尺寸: {frame.nbytes/1024:.1f}KB"
                #     )

        except (BrokenPipeError, OSError) as e:
            self.get_logger().error(f"写入失败: {str(e)}")
            self._restart_ffmpeg()
        except Exception as e:
            self.get_logger().error(f"未知写入错误: {str(e)}")
            self._restart_ffmpeg()

    def _restart_ffmpeg(self):
        """安全重启FFmpeg进程（改进版）"""
        self.restart_count += 1
        self.get_logger().error(f"第{self.restart_count}次重启推流进程...")
        self.destroy_node()
        os._exit(1)

    def _init_realsense(self):
        """初始化RealSense设备"""
        if not self.check_device_connected():
            raise RuntimeError("未检测到RealSense设备")

        # 配置视频流参数
        self.config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)
        self.config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)

        # 设备初始化重试逻辑
        for attempt in range(3):
            try:
                self.get_logger().info(f"尝试初始化设备 ({attempt+1}/3)")
                self.profile = self.pipeline.start(self.config)

                # 设备预热
                for _ in range(60):
                    self.pipeline.wait_for_frames()

                self.get_logger().info("设备初始化成功")
                self._setup_intrinsics()
                return

            except RuntimeError as e:
                self.get_logger().error(f"初始化失败: {str(e)}")
                time.sleep(2)
                if attempt == 2:
                    raise RuntimeError("设备初始化失败")

    def _setup_intrinsics(self):
        """配置相机内参"""
        color_profile = self.profile.get_stream(rs.stream.color)
        self.intrinsics = color_profile.as_video_stream_profile().get_intrinsics()

        # 预生成CameraInfo消息
        self.camera_info = CameraInfo()
        self.camera_info.header.frame_id = 'camera_color_optical_frame'
        self.camera_info.height = self.intrinsics.height
        self.camera_info.width = self.intrinsics.width
        self.camera_info.distortion_model = self.intrinsics.model.name
        self.camera_info.d = list(self.intrinsics.coeffs)
        self.camera_info.k = [
            self.intrinsics.fx, 0.0, self.intrinsics.ppx,
            0.0, self.intrinsics.fy, self.intrinsics.ppy,
            0.0, 0.0, 1.0
        ]

    def check_device_connected(self):
        """设备检测"""
        ctx = rs.context()
        return ctx.query_devices().size() > 0

    def publish_frames(self):
        """主处理循环"""
        last_stat_time = time.time()
        try:
            while rclpy.ok():
                # 控制采集频率
                elapsed = time.time() - last_stat_time
                if elapsed < 1/65:  # 65Hz采样缓冲
                    time.sleep(0.001)
                    continue

                # 获取帧数据
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                # 转换图像
                cv_image = np.asanyarray(color_frame.get_data())

                # ROS发布
                timestamp = self.get_clock().now().to_msg()
                img_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
                img_msg.header.stamp = timestamp
                img_msg.header.frame_id = 'camera_color_optical_frame'
                self.image_pub.publish(img_msg)

                # 发布相机信息
                self.camera_info.header.stamp = timestamp
                self.info_pub.publish(self.camera_info)

                # 推流处理
                try:
                    self.stream_queue.put_nowait(cv_image)
                except queue.Full:
                    self.dropped_frames += 1
                    # self.get_logger().warning(
                    #     f"队列丢弃帧 (累计: {self.dropped_frames})",
                    #     throttle_duration_sec=1
                    # )

                # 每5秒打印统计信息
                if time.time() - last_stat_time > 5:
                    # self.get_logger().info(
                    #     f"采集统计 | 队列占用={self.stream_queue.qsize()}/30 | "
                    #     f"丢弃={self.dropped_frames} | 重启={self.restart_count}"
                    # )
                    last_stat_time = time.time()

        finally:
            self.streaming_active = False
            self.pipeline.stop()
            if self.ffmpeg_proc.poll() is None:
                self.ffmpeg_proc.stdin.close()
                self.ffmpeg_proc.terminate()
            self.writer_thread.join(timeout=5)
            self.stderr_thread.join(timeout=2)
            self.get_logger().info("已释放所有资源")

def main(args=None):
    rclpy.init(args=args)
    publisher = RealsensePublisher()

    try:
        publisher.get_logger().info("开始发布和推流...")
        publisher.publish_frames()
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()