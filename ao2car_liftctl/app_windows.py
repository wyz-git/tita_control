import sys
import subprocess
import threading
import time
from typing import Optional
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QLabel, QLineEdit, QPushButton,
                            QSpinBox, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
                            QScrollArea, QStatusBar, QMessageBox, QSizePolicy, QDialog) 
from PyQt5.QtCore import Qt, QObject, pyqtSignal, QThread, QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWebEngineWidgets import QWebEngineView
import paho.mqtt.client as mqtt
import serial
from floor_control import FllorControl
from door_control import DoorControl
import numpy as np
from PyQt5.QtCore import QUrl
import signal
import atexit
import re
from PyQt5.QtWidgets import QInputDialog
import json
import psutil
import cv2

# 新增视频工作线程类
class VideoThread(QThread):
    frame_ready = pyqtSignal(np.ndarray)  # 帧数据信号
    error_occurred = pyqtSignal(str)     # 错误信号

    def __init__(self, url):
        super().__init__()
        self.url = url
        self.running = True
        self.cap = None

    def run(self):
        while self.running:
            try:
                if not self.cap or not self.cap.isOpened():
                    self.cap = cv2.VideoCapture(self.url)
                    if not self.cap.isOpened():
                        # self.error_occurred.emit("无法连接视频流")
                        time.sleep(3)  # 等待3秒后重试
                        continue

                ret, frame = self.cap.read()
                if ret:
                    self.frame_ready.emit(frame)
                else:
                    self.cap.release()
            except Exception as e:
                self.error_occurred.emit(f"视频流错误: {str(e)}")
                time.sleep(1)

    def stop(self):
        self.running = False
        if self.cap and self.cap.isOpened():
            self.cap.release()

class AudioStreamManager:
    def __init__(self):
        self.process = None
        self.restart_flag = threading.Event()
        self.stop_flag = threading.Event()
        self.error_pattern = re.compile(r"REJECT reported from HS processing")
        
        # 启动监控线程
        self.monitor_thread = threading.Thread(target=self._monitor_stream, daemon=True)
        self.monitor_thread.start()

    def _start_process(self):
        """启动ffplay进程"""
        cmd = [
            "ffplay",
            "-nodisp",    # 禁用视频显示
            "-autoexit",  # 播放结束后自动退出
            "-vn",        # 禁用视频解码
            "-af", "volume=2",
            "-loglevel", "quiet",  # 最小化日志输出
            "-fflags", "nobuffer",
            "-flags", "low_delay",
            "-analyzeduration", "0",
            "-sync", "ext",
            "-f", "mpegts",
            "srt://119.23.220.15:8890?streamid=read:tita3037207-video"
        ]
        self.process = subprocess.Popen(
            cmd,
            stderr=subprocess.PIPE,
            universal_newlines=True  # 确保文本模式读取
        )

    def _monitor_stream(self):
        """监控线程主循环"""
        while not self.stop_flag.is_set():
            if not self.process:
                self._start_process()
                
            # 实时读取stderr
            for line in iter(self.process.stderr.readline, ''):
                if self.error_pattern.search(line):
                    print("检测到连接拒绝错误，触发重启")
                    self.restart_flag.set()
                    break
                    
            # 检查进程状态
            if self.stop_flag.is_set():
                return

            retcode = self.process.poll()
            if retcode is not None:
                print(f"进程异常退出，代码: {retcode}")
                self.restart_flag.set()
                
            # 处理重启
            if self.restart_flag.is_set():
                if self.process:
                    self.process.terminate()
                    self.process.wait()
                    self.process = None
                self.restart_flag.clear()
                time.sleep(1)  # 重启间隔

    def shutdown(self):
        """彻底终止所有相关进程"""
        self.stop_flag.set()

        # 终止当前进程
        if self.process:
            try:
                parent = psutil.Process(self.process.pid)
                children = parent.children(recursive=True)  # 获取所有子进程
                for child in children:
                    child.kill()
                parent.kill()
            except psutil.NoSuchProcess:
                pass
            self.process = None

class MqttLoginDialog(QDialog):
    login_success = pyqtSignal(mqtt.Client, dict)   # 传递连接配置的信号

    def __init__(self):
        super().__init__()
        self.setWindowTitle("用户登录")
        self.setFixedSize(300, 250)
        
        layout = QVBoxLayout()
        
        # 服务器地址
        self.host_input = QLineEdit("119.23.220.15")
        layout.addWidget(QLabel("MQTT服务器:"))
        layout.addWidget(self.host_input)
        
        # 端口
        self.port_input = QSpinBox()
        self.port_input.setRange(1, 65535)
        self.port_input.setValue(1883)
        layout.addWidget(QLabel("端口:"))
        layout.addWidget(self.port_input)
        
        # 用户名
        self.username_input = QLineEdit("tita3037207")
        layout.addWidget(QLabel("用户名:"))
        layout.addWidget(self.username_input)
        
        # 密码
        self.password_input = QLineEdit()
        self.password_input.setEchoMode(QLineEdit.Password)
        self.password_input.setText("12345678")
        layout.addWidget(QLabel("密码:"))
        layout.addWidget(self.password_input)
        
        # 登录按钮
        self.login_btn = QPushButton("登录")
        self.login_btn.clicked.connect(self.attempt_login)
        layout.addWidget(self.login_btn)
        
        # 状态提示
        self.status_label = QLabel()
        layout.addWidget(self.status_label)
        
        self.setLayout(layout)

    def get_config(self):
        return {
            "host": self.host_input.text(),
            "port": self.port_input.value(),
            "username": self.username_input.text(),
            "password": self.password_input.text()
        }

    def attempt_login(self):
        """尝试连接MQTT服务器"""
        config = {
            "host": self.host_input.text(),
            "port": self.port_input.value(),
            "username": self.username_input.text(),
            "password": self.password_input.text()
        }
        
        # 创建临时客户端
        client = mqtt.Client()
        client.username_pw_set(config["username"], config["password"])
        
        # 显示连接中状态
        self.status_label.setText("正在连接服务器...")
        self.login_btn.setEnabled(False)
        
        # 异步连接检测
        def on_connect(client, userdata, flags, rc):
            client.disconnect()
            if rc == 0:
                client.loop_stop()  # 停止临时循环但保留连接
                self.login_success.emit(client, config)  # 发送客户端及配置
                self.accept()
            else:
                error_msg = {
                    1: "协议版本错误",
                    2: "客户端ID无效",
                    3: "服务器不可用",
                    4: "用户名密码错误",
                    5: "未授权"
                }.get(rc, "未知错误")
                self.status_label.setText(f"连接失败: {error_msg}")
                self.login_btn.setEnabled(True)
        
        client.on_connect = on_connect
        
        try:
            client.connect_async(config["host"], config["port"], 60)
            client.loop_start()
        except Exception as e:
            self.status_label.setText(f"连接错误: {str(e)}")
            self.login_btn.setEnabled(True)

class SerialBridgeWorker(QObject):
    data_received = pyqtSignal(bytes)
    status_changed = pyqtSignal(str)
    
    def __init__(self, serial_config, mqtt_client, topic):
        super().__init__()
        self.serial_config = serial_config
        self.mqtt_client = mqtt_client  # 传入主客户端
        self.topic = topic
        self.running = False
        self.ser = None
            
    def start(self):
        self.running = True
        try:
            # 初始化串口
            self.ser = serial.Serial(**self.serial_config)
            self.status_changed.emit("桥接已启动")
            
            # 主循环
            while self.running:
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    self.mqtt_client.publish(self.topic, data) 
                    self.data_received.emit(data)
                time.sleep(0.01)
                
        except Exception as e:
            self.status_changed.emit(f"桥接错误: {str(e)}")
        finally:
            self.stop()

    def stop(self):
        self.running = False
        self.status_changed.emit("桥接已停止")

class VideoStreamThread(QThread):
    frame_ready = pyqtSignal(QImage)
    
    def __init__(self, url):
        super().__init__()
        self.url = url
        self.running = False
        self.process = None
        self.width, self.height = 640, 480

    def run(self):
        self.running = True
        cmd = [
            "ffmpeg",
            "-rtsp_transport", "tcp",
            "-i", self.url,
            "-f", "image2pipe",
            "-pix_fmt", "rgb24",
            "-vcodec", "rawvideo",
            "-"
        ]
        try:
            self.process = subprocess.Popen(cmd, stdout=subprocess.PIPE)
            while self.running:
                raw_frame = self.process.stdout.read(self.width * self.height * 3)
                if raw_frame:
                    image = QImage(raw_frame, self.width, self.height, QImage.Format_RGB888)
                    self.frame_ready.emit(image)
        except Exception as e:
            print(f"视频流错误: {str(e)}")
        finally:
            if self.process:
                self.process.terminate()

    def stop(self):
        self.running = False
        if self.process:
            self.process.terminate()

class ElevatorControlApp(QMainWindow):
    update_status_signal = pyqtSignal(int, str, int)

    def __init__(self,mqtt_client,mqtt_config):
        super().__init__()
        self.setStyleSheet("""
                /* 全局样式 */
                QWidget {
                    font-family: '微软雅黑';
                    font-size: 12px;
                    color: #333333;
                }
                
                /* 主背景 */
                QMainWindow {
                    background-color: #F5F5F7;
                }
                
                /* 组框样式 */
                QGroupBox {
                    border: 1px solid #1976D2;
                    border-radius: 4px;
                    margin-top: 20px;
                    padding-top: 15px;
                }
                
                QGroupBox::title {
                    subcontrol-origin: margin;
                    left: 10px;
                    color: #2C3E50;
                    font-weight: bold;
                }
                
                /* 按钮基础样式 */
                QPushButton {
                    background-color: #FFFFFF;
                    border: 1px solid #D0D0D0;
                    border-radius: 4px;
                    padding: 6px 12px;
                    min-width: 80px;
                }
                
                QPushButton:hover {
                    background-color: #F8F9FA;
                    border-color: #C0C0C0;
                }
                
                QPushButton:pressed {
                    background-color: #EBECEE;
                }
                
                /* 特殊按钮样式 */
                #special_btn {
                    background-color: #3498DB;
                    color: white;
                    border: none;
                }
                
                #special_btn:hover {
                    background-color: #2980B9;
                }
                
                /* 输入框样式 */
                QLineEdit {
                    background-color: #FFFFFF; /* 浅灰色背景 */
                    border: 1px solid #000000;
                    border-radius: 3px;
                    padding: 5px;
                }
                
                QLineEdit:focus {
                    border-color: #3498DB;
                }
                
                /* 状态标签 */
                QLabel[status="normal"] {
                    color: #27AE60;
                    font-weight: 500;
                }
                
                QLabel[status="warning"] {
                    color: #E74C3C;
                    font-weight: 500;
                }
                
                /* 开关按钮 */
                QPushButton[switch="true"] {
                    background-color: #ECF0F1;
                    border: 2px solid #BDC3C7;
                }
                
                QPushButton[switch="true"]:checked {
                    background-color: #2ECC71;
                    border-color: #27AE60;
                }
                
                /* 滚动条 */
                QScrollArea {
                    border: none;
                    background: transparent;
                }
                
                QScrollBar:vertical {
                    width: 10px;
                    background: #F0F0F0;
                }
                
                QScrollBar::handle:vertical {
                    background: #C4C4C4;
                    min-height: 20px;
                    border-radius: 5px;
                }
            """)
        self.common_phrases = ["小姐姐你真好看", "我可以要一个微信吗", "谢谢您", "请问我的奶茶好了吗","哈喽，让让我一下"]
        self.switch_buttons = []
        self.mqtt_config = mqtt_config  # 存储登录配置
        self.device_id = mqtt_config['username']
        self.mqtt_client = mqtt_client
        self.setWindowTitle("TITA控制端")
        self.setGeometry(100, 100, 1280, 720)
        self.update_status_signal.connect(self._update_elevator_status)
        # 初始化组件
        self.init_ui()
        self.init_connections()
        self.init_workers()
        self.audio_manager = AudioStreamManager()

    def _update_elevator_status(self, floor, motion, door_state):
        """更新电梯状态显示"""
        motion_map = {
            "STOP": "待机",
            "UP": "上行▲",
            "DOWN": "下行▼"
        }
        door_status = "开启" if door_state == 4 else "关闭"
        # 更新状态监控标签样式
        self.floor_var.setText(f"{floor}F")
        self.motion_var.setText(motion_map.get(motion, "未知状态"))
        self.door_var.setText(door_status)
        
    def start_door_control(self):
        """启动门禁控制线程"""
        """门控后台任务"""
        try:
            main_instance = DoorControl()
            response = main_instance.door_control()
            
        #     # 通过after方法安全更新UI（网页7通信机制）
        #     self.root.after(0, lambda: self._update_door_status(response))
        except Exception as e:
            # self.root.after(0, lambda: self.update_status(f"门控异常: {str(e)}"))
            print(f"start_door_control 错误: {str(e)}")

    def _update_door_status(self, status):
        """更新门状态显示"""
        self.door_var.setText(status)
        self.status_bar.showMessage("门控操作完成", 3000)

    def _handle_door_error(self, error_msg):
        """处理门控错误"""
        QMessageBox.critical(self, "门控异常", error_msg)
        self.status_bar.showMessage("操作失败", 3000)
        
    def init_ui(self):
        # 主布局
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        
        # 左侧控制面板
        control_panel = self.create_control_panel()
        main_layout.addWidget(control_panel, stretch=3)
        
        # 右侧视频面板
        video_panel = self.create_video_panel()
        main_layout.addWidget(video_panel, stretch=7)
        
        # 状态栏
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)

    def publish_switch_states(self):
        """发布开关状态到MQTT"""
        if not self.mqtt_client.is_connected():
            QMessageBox.warning(self, "警告", "MQTT连接未就绪")
            return

        # 构建JSON数据
        switch_states = {
            str(i+1): btn.isChecked()
            for i, btn in enumerate(self.switch_buttons)
        }
        
        try:
            payload = json.dumps(switch_states, ensure_ascii=False)
            topic = f"{self.device_id}/switches"
            self.mqtt_client.publish(topic, payload)
            self.status_bar.showMessage("开关状态已发布", 2000)
        except Exception as e:
            QMessageBox.critical(self, "发布错误", f"MQTT发布失败: {str(e)}")

    def create_control_panel(self):
        panel = QScrollArea()
        content = QWidget()
        layout = QVBoxLayout(content)
        
        # 电梯控制
        elevator_group = QGroupBox("电梯控制")
        elevator_layout = QGridLayout()
        self.start_floor = QSpinBox()
        self.exit_floor = QSpinBox()
        self.start_floor.setRange(1, 20)
        self.exit_floor.setRange(1, 20)
        elevator_layout.addWidget(QLabel("起始楼层:"), 0, 0)
        elevator_layout.addWidget(self.start_floor, 0, 1)
        elevator_layout.addWidget(QLabel("目标楼层:"), 1, 0)
        elevator_layout.addWidget(self.exit_floor, 1, 1)

        elevator_group.setLayout(elevator_layout)
        layout.addWidget(elevator_group)

        self.btn_elevator = QPushButton("启动电梯控制")
        self.btn_door = QPushButton("门禁控制")
        self.btn_door.setObjectName("special_btn")
        self.btn_elevator.setObjectName("special_btn")
        # 按钮容器
        button_container = QHBoxLayout()
        button_container.addWidget(self.btn_elevator)
        button_container.addWidget(self.btn_door)
        
        elevator_layout.addLayout(button_container, 2, 0, 1, 2)  # 跨两列
        
        elevator_group.setLayout(elevator_layout)
        layout.addWidget(elevator_group)

        # 添加状态监控组
        status_group = QGroupBox("状态监控")
        status_layout = QGridLayout()
        
        # 楼层状态
        self.floor_var = QLabel("N/A")
        status_layout.addWidget(QLabel("当前楼层:"), 0, 0)
        status_layout.addWidget(self.floor_var, 0, 1)
        
        # 运行状态
        self.motion_var = QLabel("N/A")
        status_layout.addWidget(QLabel("运行状态:"), 1, 0)
        status_layout.addWidget(self.motion_var, 1, 1)
        
        # 门状态
        self.door_var = QLabel("N/A")
        status_layout.addWidget(QLabel("门状态:"), 2, 0)
        status_layout.addWidget(self.door_var, 2, 1)

        self.floor_var.setProperty("status", "normal")
        self.motion_var.setProperty("status", "normal") 
        self.door_var.setProperty("status", "normal")

        status_group.setLayout(status_layout)
        layout.addWidget(status_group)


        # 串口桥接
        serial_group = QGroupBox("串口桥接")
        serial_layout = QGridLayout()
        self.serial_port = QLineEdit("COM11")
        self.btn_bridge = QPushButton("启动桥接")
        self.btn_bridge.setObjectName("special_btn")
        serial_layout.addWidget(QLabel("串口端口:"), 0, 0)
        serial_layout.addWidget(self.serial_port, 0, 1)
        serial_layout.addWidget(self.btn_bridge, 1, 0, 1, 2)  # 调整按钮位置
        serial_group.setLayout(serial_layout)
        
        # 修改消息发送部分的布局
        message_group = QGroupBox("消息通信")
        message_layout = QVBoxLayout()
        
        # 创建垂直滚动区域
        self.phrase_scroll = QScrollArea()
        self.phrase_scroll.setWidgetResizable(True)
        self.phrase_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.phrase_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.phrase_scroll.setFixedHeight(150)  # 固定高度
        
        self.phrase_container = QWidget()
        self.phrase_layout = QVBoxLayout(self.phrase_container)  # 改为垂直布局
        self.phrase_layout.setContentsMargins(5, 5, 5, 5)
        self.phrase_layout.setSpacing(3)
        
        # 初始化短语按钮
        self.refresh_phrase_buttons()
        
        # 添加按钮放在底部
        add_btn = QPushButton("＋ 添加常用短语")
        add_btn.clicked.connect(self.add_common_phrase)
        add_btn.setStyleSheet("""
            QPushButton {
                background-color: #f0f0f0;
                border: 1px solid #cccccc;
                padding: 5px;
            }
            QPushButton:hover { background-color: #e0e0e0; }
        """)
        self.phrase_layout.addWidget(add_btn)
        
        self.phrase_scroll.setWidget(self.phrase_container)
        message_layout.addWidget(self.phrase_scroll)

        self.message_input = QLineEdit()
        self.btn_send = QPushButton("发送消息")
        # 在消息发送按钮添加特殊样式
        self.btn_send.setObjectName("special_btn")
        message_layout.addWidget(self.message_input)
        message_layout.addWidget(self.btn_send)

        # ===== 新增开关按钮区域 =====
        # 开关按钮容器
        switch_container = QWidget()
        switch_layout = QGridLayout(switch_container)
        switch_layout.setContentsMargins(0, 10, 0, 0)  # 上边距10px
        
        # 创建10个开关按钮
        self.switch_buttons = []
        for i in range(10):
            if i == 0:
                btn = QPushButton(f"自动巡检")
            elif i == 1:
                btn = QPushButton(f"远程巡检")
            else:
                btn = QPushButton(f"switch {i+1}")
            btn.setCheckable(True)
            btn.setProperty("switch", "true")  # 添加自定义属性
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #F5F5F5;
                    border: 2px solid #000000;
                    border-radius: 4px;
                    padding: 6px;
                    min-width: 80px;
                }
                QPushButton:checked {
                    background-color: #4CAF50;
                    border-color: #45A049;
                }
            """)
            btn.clicked.connect(self.publish_switch_states)
            self.switch_buttons.append(btn)
            # 布局：5列x2行
            switch_layout.addWidget(btn, i//5, i%5) 
        
        # 将开关容器添加到消息布局底部
        message_layout.addWidget(switch_container)

        message_group.setLayout(message_layout)
        layout.addWidget(elevator_group)
        layout.addWidget(serial_group)
        layout.addWidget(message_group)
        layout.addStretch()
        
        panel.setWidget(content)
        panel.setWidgetResizable(True)
        return panel

    def refresh_phrase_buttons(self):
        """刷新垂直排列的短语按钮"""
        # 清除旧按钮（保留最后的添加按钮）
        while self.phrase_layout.count() > 1:
            widget = self.phrase_layout.itemAt(0).widget()
            if widget:
                widget.deleteLater()
                self.phrase_layout.removeWidget(widget)

        # 添加新按钮（在添加按钮之前插入）
        for phrase in self.common_phrases:
            btn = QPushButton(phrase)
            btn.setStyleSheet("""
                QPushButton {
                    text-align: left;
                    padding: 8px;
                    border: 1px solid #EDEDED;
                    background-color: #FFFFFF;
                    color: #2C3E50;
                }
                QPushButton:hover {
                    background-color: #F8F9FA;
                }
            """)

            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            btn.clicked.connect(lambda _, p=phrase: self.message_input.setText(p))
            self.phrase_layout.insertWidget(0, btn)  # 插入到顶部

    def add_common_phrase(self):
        """添加常用短语"""
        new_phrase, ok = QInputDialog.getText(
            self, 
            "添加短语", 
            "输入新短语:",
            QLineEdit.Normal
        )
        
        if ok and new_phrase:
            if new_phrase not in self.common_phrases:
                self.common_phrases.append(new_phrase)
                self.refresh_phrase_buttons()
                
                # 可选：保存到配置文件（需要添加持久化逻辑）
                # self.save_phrases_to_config()

    def create_video_panel(self):
        panel = QWidget()
        # 设置面板的布局策略（关键点1：确保父容器可扩展）
        panel.setSizePolicy(
            QSizePolicy.Expanding,  # 水平方向尽可能扩展
            QSizePolicy.Expanding   # 垂直方向尽可能扩展
        )
        
        # 使用无间距布局（关键点2：消除内部边距）
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)  # 四周边距归零
        layout.setSpacing(0)                   # 控件间距归零

        # 配置视频显示标签（关键点3：自动缩放内容）
        self.video_label = QLabel()
        self.video_label.setSizePolicy(
            QSizePolicy.Expanding,  # 水平填满
            QSizePolicy.Expanding   # 垂直填满
        )
        self.video_label.setAlignment(Qt.AlignCenter)  # 内容居中（可选）
        self.video_label.setScaledContents(True)       # 关键：允许自动缩放
        
        # 将标签添加到布局（关键点4：设置拉伸因子）
        layout.addWidget(self.video_label, stretch=1)  # stretch=1 表示占据全部剩余空间

        # 初始化视频线程（统一使用正确端口）
        stream_url = f"rtsp://119.23.220.15:8554/{self.device_id}"  # 确认端口一致性
        self.video_thread = VideoThread(stream_url)
        self.video_thread.frame_ready.connect(self.update_frame)
        self.video_thread.error_occurred.connect(self.handle_stream_error)
        self.video_thread.start()

        # 创建定时器（每5秒自动刷新）
        self.refresh_timer = QTimer()
        self.refresh_timer.setInterval(5000)  # 5秒
        self.refresh_timer.timeout.connect(self.reconnect_timer)
        self.refresh_timer.start()  # 启动定时器

        return panel

    def update_frame(self, frame):
        try:
            # 转换颜色空间
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            
            # 创建并显示QImage
            qt_image = QImage(rgb_image.data, w, h, ch*w, QImage.Format_RGB888)
            self.video_label.setPixmap(QPixmap.fromImage(qt_image))
        except Exception as e:
            print(f"帧处理错误: {str(e)}")

    def handle_stream_error(self, msg):
        # 显示错误提示
        QMessageBox.warning(self, "视频流中断", msg)
        
        # 安全重启视频线程
        if self.video_thread.isRunning():
            self.video_thread.stop()
            self.video_thread.wait()
        
        # 延迟2秒后重连
        QTimer.singleShot(2000, self.restart_stream)

    def restart_stream(self):
        self.video_thread = VideoThread(f"rtsp://119.23.220.15:8554/{self.device_id}")
        self.video_thread.frame_ready.connect(self.update_frame)
        self.video_thread.error_occurred.connect(self.handle_stream_error)
        self.video_thread.start()

    def reconnect_timer(self):
        self.publish_switch_states()

    def on_page_load_finished(self, success):
        """页面加载完成回调"""
        if not success:
            print("检测到加载失败，触发紧急重连...")
            self.reconnect_video()

    def init_connections(self):
        self.btn_elevator.clicked.connect(self.start_elevator)
        self.btn_bridge.clicked.connect(self.toggle_bridge)
        self.btn_send.clicked.connect(self.send_message)
        self.message_input.returnPressed.connect(self.send_message)
        # 新增门禁按钮连接
        self.btn_door.clicked.connect(self.start_door_control)

    def init_workers(self):
        self.bridge_thread = QThread()
        self.bridge_worker = None

    def start_elevator(self):
        try:
            start = self.start_floor.value()
            exit = self.exit_floor.value()
            if start == exit:
                raise ValueError("起始与目标楼层相同")
            
            controller = FllorControl(gui_instance=self,start_floor=start, exit_floor=exit)
            threading.Thread(target=controller.test_run, daemon=True).start()
            self.status_bar.showMessage(f"电梯控制启动: {start}F → {exit}F")
        except Exception as e:
            QMessageBox.critical(self, "错误", str(e))

    def toggle_bridge(self):
        if self.bridge_worker and self.bridge_worker.running:
            self.stop_bridge()
        else:
            self.start_bridge()

    def start_bridge(self):
        bridge_config = {
            "serial": {
                "port": self.serial_port.text(),
                "baudrate": 420000,
                "timeout": 1
            },
            "topic": f"{self.device_id}/control"
        }
        
        # 传入主客户端和配置
        self.bridge_worker = SerialBridgeWorker(
            serial_config=bridge_config['serial'],
            mqtt_client=self.mqtt_client,
            topic=bridge_config['topic']
        )
        self.bridge_worker.moveToThread(self.bridge_thread)
        self.bridge_thread.started.connect(self.bridge_worker.start)
        self.bridge_worker.status_changed.connect(self.status_bar.showMessage)
        self.bridge_thread.start()
        self.btn_bridge.setText("停止桥接")

    def stop_bridge(self):
        if self.bridge_worker:
            self.bridge_worker.stop()
            self.bridge_thread.quit()
            self.btn_bridge.setText("启动桥接")

    def send_message(self):
        message = self.message_input.text()
        if message:
            self.mqtt_client.publish(
                topic=f"{self.device_id}/messages",
                payload=message.encode()
            )
            self.status_bar.showMessage(f"消息已发送至设备{self.device_id}") 

    def closeEvent(self, event):
        self.stop_bridge()
        self.audio_manager.shutdown()
        if self.video_thread:
            self.video_thread.stop()
            self.video_thread.quit()  # 发送退出信号
            self.video_thread.wait()
        super().closeEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    login_dialog = MqttLoginDialog()
    if login_dialog.exec_() == QDialog.Accepted:
        config = login_dialog.get_config()
        # 获取客户端和配置
        client = mqtt.Client()
        client.username_pw_set(config["username"], config["password"])
        client.connect(config["host"], config["port"])
        client.loop_start()
        window = ElevatorControlApp(client, config)

        # 窗口显示三步曲
        window.showMaximized()     # 最大化显示
        window.activateWindow()   # 激活窗口
        window.raise_()           # 提升到顶层
        
        # 强制刷新界面
        QApplication.processEvents()
        
        sys.exit(app.exec_())
    else:
        sys.exit(0)