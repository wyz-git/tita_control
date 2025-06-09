#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import time
import signal
import logging
import json
import os
from pathlib import Path

class CameraControl(Node):
    def __init__(self):
        super().__init__('camera_controller')

        # 加载配置文件
        self.config = self.load_config()
        self.gst_process = None

        # 配置日志
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        self.logger = logging.getLogger(self.get_name())

        # 创建订阅
        self.subscription_switch = self.create_subscription(
            String,
            f"/{self.config['mqtt_username']}/switch_states",
            self.switch_states_callback,
            10
        )
        self.axes_mode = False
        self.axes_mode_old = False
        self.logger.info("节点已初始化，等待开关状态指令...")

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

    def parse_switch_state(self, data):
        """
        解析开关状态消息，支持多种格式：
        - JSON 格式: {"4": true, ...}
        - 键值对格式: "4:true,5:false"
        """
        try:
            # 尝试解析 JSON
            import json
            state_dict = json.loads(data.data)
            return state_dict.get("4", False)
        except Exception as json_error:
            self.logger.debug("JSON 解析失败，尝试键值对解析: %s", json_error)
            # 回退到键值对解析
            pairs = data.data.split(',')
            for pair in pairs:
                if ':' in pair:
                    key, value = pair.split(':', 1)
                    if key.strip() == "4":
                        return value.strip().lower() == "true"
            return False

    def safe_execute_command(self, command, timeout=10):
        """安全执行 shell 命令并返回结果"""
        try:
            result = subprocess.run(
                command,
                shell=True,
                check=True,
                timeout=timeout,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            self.logger.debug("命令执行成功: %s\n输出: %s", command, result.stdout)
            return True
        except subprocess.CalledProcessError as e:
            self.logger.error("命令执行失败: %s\n错误: %s", command, e.stderr)
            return False
        except Exception as e:
            self.logger.error("执行命令时发生意外错误: %s", str(e))
            return False

    def kill_camera_process(self):
        """终止相机进程（无需密码）"""
        try:
            # 获取 PID
            pgrep_result = subprocess.run(
                ["pgrep", "-f", "argus_camera_device"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            if pgrep_result.returncode != 0:
                self.logger.warning("未找到目标进程")
                return True

            pids = pgrep_result.stdout.strip().split()
            if not pids:
                return True

            # 终止进程
            kill_command = ["sudo", "-n", "/usr/bin/kill", "-9"] + pids
            result = subprocess.run(
                kill_command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )

            if result.returncode == 0:
                self.logger.info("成功终止进程 PID: %s", pids)
                return True
            else:
                self.logger.error("终止失败: %s", result.stderr)
                return False

        except Exception as e:
            self.logger.error("进程终止异常: %s", str(e))
            return False

    def start_gstreamer_stream(self):
        """启动 GStreamer 推流进程"""
        gst_cmd = [
            'gst-launch-1.0', '-v',
            'nvarguscamerasrc', 'sensor-id=0',
            '!', 'video/x-raw(memory:NVMM),width=1920,height=1200,format=NV12,framerate=60/1',
            '!', 'nvvidconv',
            '!', 'video/x-raw,width=960,height=600,format=I420',
            '!', 'x264enc', 'tune=zerolatency', 'speed-preset=ultrafast',
            '!', 'queue',
            '!', 'video/x-h264,profile=baseline',
            '!', 'h264parse',
            '!', 'tee', 'name=t',
            '!', 'queue',
            '!', 'rtspclientsink', 'name=sink1', 'location=rtsp://127.0.0.1:8554/tita', 'protocols=tcp',
            't.',
            '!', 'queue',
            '!', 'rtspclientsink', 'name=sink2', f"location=rtsp://119.23.220.15:8554/{self.config['mqtt_username']}", 'protocols=tcp'
        ]

        try:
            # 终止可能存在的旧进程
            if self.gst_process and self.gst_process.poll() is None:
                self.logger.warning("发现正在运行的 GStreamer 进程，先终止")
                self.gst_process.terminate()
                self.gst_process.wait(timeout=5)

            # 启动新进程
            self.gst_process = subprocess.Popen(
                gst_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid  # 创建新进程组
            )
            self.logger.info("GStreamer 推流已启动，PID: %d", self.gst_process.pid)
            return True
        except Exception as e:
            self.logger.error("启动 GStreamer 失败: %s", str(e))
            return False

    def switch_states_callback(self, msg):
        switch_states = json.loads(msg.data)
        self.axes_mode = switch_states.get("3", 0)
        if self.axes_mode != self.axes_mode_old:
            self.axes_mode_old = self.axes_mode
            if self.axes_mode:
                # True 分支处理
                self.logger.info("检测到开关4为 True，执行重启推流流程")

                # 终止进程
                if self.kill_camera_process():
                    self.logger.info("成功终止相机进程")
                else:
                    self.logger.error("终止相机进程失败，继续后续操作")

                # 等待 3 秒
                self.logger.info("等待 3 秒...")
                time.sleep(3)

                # 启动推流
                if self.start_gstreamer_stream():
                    self.logger.info("推流启动成功")
                else:
                    self.logger.error("推流启动失败")
            else:
                # 1. 终止 GStreamer 推流进程
                if self.gst_process and self.gst_process.poll() is None:
                    try:
                        self.logger.info("正在终止 GStreamer 推流进程 PID: %d", self.gst_process.pid)
                        self.gst_process.send_signal(signal.SIGINT)  # 优雅终止
                        self.gst_process.wait(timeout=5)
                        self.logger.info("推流进程已终止")
                    except subprocess.TimeoutExpired:
                        self.logger.warning("正常终止超时，强制杀死进程")
                        self.gst_process.kill()
                        self.gst_process.wait()
                    except Exception as e:
                        self.logger.error("终止推流进程失败: %s", str(e))

                # False 分支处理
                self.logger.info("检测到开关4为 False，执行服务重启")
                result = self.safe_execute_command("sudo systemctl restart tita-perception.service")
                if result:
                    self.logger.info("服务重启成功")
                else:
                    self.logger.error("服务重启失败")

    def __del__(self):
        """析构函数确保清理资源"""
        if self.gst_process and self.gst_process.poll() is None:
            self.logger.info("终止正在运行的 GStreamer 进程")
            self.gst_process.terminate()
            self.gst_process.wait()

def main(args=None):
    rclpy.init(args=args)

    # 初始化节点
    controller = CameraControl()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理工作
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
