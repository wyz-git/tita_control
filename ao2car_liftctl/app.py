import tkinter as tk
from tkinter import ttk, messagebox
import subprocess
import threading
import paho.mqtt.client as mqtt
import serial
import time
from typing import Optional
from floor_control import FllorControl
from door_control import DoorControl
from PIL import Image, ImageTk

class SerialToMqttBridge:
    def __init__(self, serial_port, baudrate, mqtt_server, mqtt_port, mqtt_topic):
        self.serial_config = {
            "port": serial_port,
            "baudrate": baudrate,
            "timeout": 1
        }
        self.mqtt_config = {
            "server": mqtt_server,
            "port": mqtt_port,
            "topic": mqtt_topic,
            "client_id": "serial_to_mqtt"
        }
        self.serial_buffer = bytearray()
        self.running = True
        self.ser = None
        self.mqtt_client = None
        self._init_connections()

    def _init_connections(self):
        try:
            self.ser = serial.Serial(
                port=self.serial_config["port"],
                baudrate=self.serial_config["baudrate"],
                timeout=self.serial_config["timeout"]
            )
            self.mqtt_client = mqtt.Client(client_id=self.mqtt_config["client_id"])
            self.mqtt_client.connect(
                host=self.mqtt_config["server"],
                port=self.mqtt_config["port"],
                keepalive=60
            )
            self.mqtt_client.loop_start()
        except Exception as e:
            raise Exception(f"初始化失败: {str(e)}")

    def run(self):
        try:
            while self.running and self.ser and self.ser.is_open:
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    self.serial_buffer.extend(data)
                self._process_serial_data()
                time.sleep(0.01)
        finally:
            self._cleanup()

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
            
    def stop(self):
        self.running = False

    def _cleanup(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()

class DualSRTPlayerWithMQTT:
    def __init__(self, root):
        self.root = root
        self.root.title("TITA控制端")
    
        # 初始化变量
        self.process1 = None  # 主SRT流进程
        self.process2 = None  # 备SRT流进程
        self.mqtt_client = None
        self.playing1 = False  # 主流状态
        self.playing1_stop = True  # 主流状态
        self.playing2 = False  # 备流状态
        self.bridge = None
        self.bridge_thread = None
        self.last_frame_time = time.time()
        self.create_parameter_inputs()
        self.create_video_display()
        self.create_widgets()
        self._init_styles() 
        # self.root.after(1000, self._start_read_check_timer) 
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)


    def _start_read_check_timer(self):
        """启动读取状态检查定时器"""
        if not self.playing1:  # 只在主流运行时工作
            self.read_check_timer = self.root.after(1000, self._start_read_check_timer)
            return
        
        if time.time() - self.last_frame_time > 8:
            self._restart_stream(1)
            self.last_frame_time = time.time()

        
        # 每秒检查一次
        self.read_check_timer = self.root.after(1000, self._start_read_check_timer)

    def _stop_read_check_timer(self):
        """停止定时器"""
        if self.read_check_timer:
            self.root.after_cancel(self.read_check_timer)
            self.read_check_timer = None

    def _init_styles(self):
        """初始化界面样式"""
        style = ttk.Style()
        style.theme_use('clam')
        
        # 配置全局按钮样式
        style.configure('TButton', 
                       background='#409EFF',  # 主色调
                       foreground='white',
                       font=('微软雅黑', 10),
                       borderwidth=0,
                       relief='flat',
                       padding=6)
        
        # 按钮交互状态颜色变化
        style.map('TButton',
                 background=[('active', '#337ECC'),  # 悬停颜色
                            ('disabled', '#C0C4CC')], # 禁用状态
                 foreground=[('disabled', '#FFFFFF')])
        
        # 特殊按钮样式
        style.configure('Primary.TButton',  # 重要操作按钮
                      background='#67C23A',
                      foreground='white')
        style.configure('Danger.TButton',  # 危险操作按钮
                      background='#F56C6C',
                      foreground='white')
        style.configure('Phrase.TButton',  # 短语按钮
                      background='#909399',
                      foreground='white',
                      font=('微软雅黑', 9),
                      padding=3)


    def _restart_stream(self, stream_num):
        """带延迟的流重启机制"""
        self.stop_stream(stream_num)
        time.sleep(2)  # 等待2秒避免频繁重启
        self.start_stream(stream_num)
        self.update_status(f"流{stream_num} 自动重连中...")
        
    def video_decode_thread(self, process):
        width, height = 848, 480
        i1 = 0
        self.last_frame_time = time.time()
        while True:
            # 心跳检测（每2秒检查）
            if self.playing1_stop:
                print("video_decode_thread break")
                break  #
            if time.time() - self.last_frame_time > 2:
                self.root.after(0, self._restart_stream, 1)
                break
            # print("read")
            raw_frame = process.stdout.read(width * height * 3)
            # print("read ok")
            if raw_frame:
                self.last_frame_time = time.time()
                image = Image.frombytes('RGB', (width, height), raw_frame)
                # resized_img = image.resize((848, 480), Image.Resampling.NEAREST)
                self.root.after(0, self._safe_update_image, image)
                # i1=i1+1
                # print("read",i1)
            else:
                print("video_decode_thread error")
                # 进程异常退出时重启
                if process.poll() is not None:
                    self.root.after(0, self._restart_stream, 1)
                    break

    def _safe_update_image(self, image):
        self.tk_image = ImageTk.PhotoImage(image)
        self.video_label.config(image=self.tk_image)
        self.video_label.image = self.tk_image

    def create_video_display(self):
        """右上方固定尺寸视频显示面板"""
        # 主容器使用grid布局
        main_container = ttk.Frame(self.root)
        main_container.pack(fill=tk.BOTH, expand=True)

        # 视频容器（右上方）
        video_container = ttk.Frame(main_container)
        video_container.grid(row=0, column=1, sticky="ne", padx=5, pady=5)

        # 固定尺寸视频框架
        self.video_frame = ttk.LabelFrame(video_container, 
                                        text="实时视频流",
                                        width=848, 
                                        height=480)
        self.video_frame.pack_propagate(False)  # 禁止自动调整大小
        self.video_frame.pack()

        # Canvas视频画布
        self.canvas = tk.Canvas(
            self.video_frame,
            bg='black',
            width=848,
            height=480,
            highlightthickness=0
        )
        self.canvas.pack()

        # 图像标签锚定到左上角
        self.video_label = ttk.Label(self.canvas)
        self.canvas.create_window(0, 0, 
                                window=self.video_label, 
                                anchor="nw", 
                                tags="video_window")

        # 配置grid布局权重
        main_container.columnconfigure(0, weight=1)  # 左侧区域自适应
        main_container.columnconfigure(1, weight=0)  # 右侧固定宽度
        main_container.rowconfigure(0, weight=1)      # 单行布局
        

    def start_door_control(self):
        try:
            # 创建独立线程执行门控操作
            door_thread = threading.Thread(target=self._door_control_task, daemon=True)
            door_thread.start()
        except Exception as e:
            self.Log.error(f"门控线程启动失败: {str(e)}")

    def _door_control_task(self):
        """门控后台任务"""
        try:
            main_instance = DoorControl()
            response = main_instance.door_control()
            
            # 通过after方法安全更新UI（网页7通信机制）
            self.root.after(0, lambda: self._update_door_status(response))
        except Exception as e:
            self.root.after(0, lambda: self.update_status(f"门控异常: {str(e)}"))

    def _update_door_status(self, response):
        """门控结果反馈"""
        if 200 <= response.status_code < 300:
            messagebox.showinfo("门控成功", f"状态码: {response.status_code}\n响应内容: {response.text}")
        else:
            messagebox.showerror("门控失败", f"错误状态码: {response.status_code}")

    def create_parameter_inputs(self):
        """创建电梯控制面板（左半部布局）"""
        # 主容器（占左侧50%宽度）
        # 限制最小/最大宽度
        left_container = ttk.Frame(self.root)
        left_container.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # 参数输入面板
        param_frame = ttk.LabelFrame(left_container, text="电梯控制参数")
        param_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # 使用Grid布局规范输入控件
        param_frame.columnconfigure(1, weight=1)
        
        # 起始楼层
        ttk.Label(param_frame, text="起始楼层:").grid(row=0, column=0, padx=5, pady=2, sticky="e")
        self.start_floor_entry = ttk.Spinbox(param_frame, from_=1, to=20, width=8)
        self.start_floor_entry.set(1)
        self.start_floor_entry.grid(row=0, column=1, padx=5, pady=2, sticky="w")
        
        # 目标楼层
        ttk.Label(param_frame, text="目标楼层:").grid(row=1, column=0, padx=5, pady=2, sticky="e")
        self.exit_floor_entry = ttk.Spinbox(param_frame, from_=1, to=20, width=8)
        self.exit_floor_entry.set(9)
        self.exit_floor_entry.grid(row=1, column=1, padx=5, pady=2, sticky="w")
        
        # 控制按钮容器
        btn_frame = ttk.Frame(param_frame)
        btn_frame.grid(row=2, column=0, columnspan=2, pady=10)
        
        # 修改电梯控制按钮
        ttk.Button(btn_frame, text="启动电梯控制", 
                 command=self.start_elevator_control,
                 style='Primary.TButton').pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="门禁控制", 
                 command=self.start_door_control,
                 style='Primary.TButton').pack(side=tk.LEFT, padx=5)

        # 状态监控面板
        status_frame = ttk.LabelFrame(left_container, text="电梯状态监控")
        status_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # 使用Grid布局状态显示
        status_frame.columnconfigure(1, weight=1)
        # 创建StringVar变量（网页8 Variable类应用）
        self.floor_var = tk.StringVar(value="N/A")
        self.motion_var = tk.StringVar(value="N/A") 
        self.door_var = tk.StringVar(value="N/A")
        
        status_items = [
            ("当前楼层:", self.floor_var),
            ("运行状态:", self.motion_var),
            ("开启状态:", self.door_var)
        ]
        
        for row, (label, var) in enumerate(status_items):
            ttk.Label(status_frame, text=label).grid(row=row, column=0, padx=5, pady=2, sticky="e")
            ttk.Label(status_frame, textvariable=var, 
                    font=('Arial', 9, 'bold')).grid(row=row, column=1, padx=5, pady=2, sticky="w")
            
    # =================== 消息发送区域 ===================
        msg_frame = ttk.LabelFrame(left_container, text="消息发布")
        msg_frame.pack(fill=tk.X, padx=5, pady=5)

        # 新增可滑动短语栏（结合网页6的滚动条实现）
        phrase_container = ttk.Frame(msg_frame)
        phrase_container.pack(fill=tk.X, pady=3)

        # 创建水平滚动容器（参考网页7的Canvas实现）
        canvas = tk.Canvas(phrase_container, height=30, highlightthickness=0)
        scroll_x = ttk.Scrollbar(phrase_container, orient="horizontal", command=canvas.xview)
        
        # 配置滚动区域
        canvas.configure(xscrollcommand=scroll_x.set)
        scroll_x.pack(side=tk.BOTTOM, fill=tk.X)
        canvas.pack(side=tk.TOP, fill=tk.X)

        # 创建短语按钮容器（使用网页6的Frame嵌套技术）
        btn_frame = ttk.Frame(canvas)
        self.btn_frame_id = canvas.create_window((0,0), window=btn_frame, anchor="nw")  # 保存ID

        # 常用短语配置（示例20个，可扩展）
        phrases = [
            "请让让我","你好","谢谢",
            "我来拿奶茶啦","帮我开下门","奶茶帮我放上面",
            "危险请远离", "请问多少钱", "需要帮助吗", "前方拥堵", 
            "注意安全", "请勿靠近", "紧急联系", 
            "右转通行", "左转等候", "直行通过",
            "停车等待", "保持距离", "减速慢行",
            "谢谢配合", "请出示证件", "系统故障",
            "正在处理", "稍等片刻","站住,打劫"
        ]

        # 动态生成按钮（参考网页5的批量创建技术）
        for idx, text in enumerate(phrases):
            btn = ttk.Button(
                btn_frame, 
                text=text,
                width=10,
                command=lambda t=text: self.message_entry.insert(tk.END, t),
                style='Phrase.TButton'
            )
            btn.grid(row=0, column=idx, padx=2, sticky="ew")

        # 自适应配置
        btn_frame.update_idletasks()
        canvas.config(scrollregion=canvas.bbox("all"))
        # canvas.bind("<Configure>", 
        #     lambda e: canvas.itemconfig(btn_frame, width=e.width))
        
        # 输入框与发送按钮（保持原有结构）
        self.message_entry = ttk.Entry(msg_frame)
        self.message_entry.pack(fill=tk.X, padx=5, pady=2)
        self.message_entry.bind("<Return>", self.on_enter_pressed)

        self.btn_publish = ttk.Button(msg_frame, 
                                    text="发送消息", 
                                    command=self.send_message)
        self.btn_publish.pack(pady=5)

    def start_elevator_control(self):
        """启动电梯控制线程"""
        try:
            start = int(self.start_floor_entry.get())
            exit = int(self.exit_floor_entry.get())
            
            # 参数有效性验证
            if not (1 <= start <= 20 and 1 <= exit <= 20):
                raise ValueError("楼层范围1-20")
            if start == exit:
                raise ValueError("起始/目标楼层相同")

            # 多线程启动电梯控制
            elevator_main = FllorControl(gui_instance=self,start_floor=start, exit_floor=exit)
            threading.Thread(
                target=elevator_main.test_run,
                daemon=True
            ).start()
            
            self.update_status(f"电梯控制已启动: {start}F → {exit}F")
        except ValueError as e:
            error_msg = {
                "invalid literal": "楼层必须为数字",
                "楼层范围1-20": "楼层范围需在1-20之间",
                "起始/目标楼层相同": "起始与目标楼层不能相同"
            }.get(str(e), "参数输入错误")
            messagebox.showerror("输入错误", error_msg)
            self.update_status("启动失败: " + error_msg)

    def create_widgets(self):
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # 串口配置区域
        serial_frame = ttk.LabelFrame(main_frame, text="串口桥接配置")
        serial_frame.pack(fill=tk.X, padx=5, pady=5)

        ttk.Label(serial_frame, text="串口端口:").grid(row=0, column=0, padx=5, pady=2)
        self.serial_port_entry = ttk.Entry(serial_frame)
        self.serial_port_entry.insert(0, "/dev/ttyUSB0")
        self.serial_port_entry.grid(row=0, column=1, padx=5, pady=2)

        self.bridge_btn = ttk.Button(serial_frame, text="启动桥接", command=self.toggle_bridge)
        self.bridge_btn.grid(row=1, column=0, columnspan=4, pady=5)
        # 左侧控制面板
        left_panel = ttk.Frame(main_frame)
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # 右侧控制面板
        #right_panel = ttk.Frame(main_frame)
        #right_panel.pack(side=tk.RIGHT, fill=tk.Y)

        # MQTT配置区域
        mqtt_frame = ttk.LabelFrame(left_panel, text="MQTT配置")
        mqtt_frame.pack(fill=tk.X, padx=5, pady=5)

        ttk.Label(mqtt_frame, text="服务器:").grid(row=0, column=0, padx=5, pady=2)
        self.mqtt_host = ttk.Entry(mqtt_frame, width=18)
        self.mqtt_host.insert(0, "119.23.220.15")
        self.mqtt_host.grid(row=0, column=1, padx=5, pady=2)

        ttk.Label(mqtt_frame, text="端口:").grid(row=0, column=2, padx=5, pady=2)
        self.mqtt_port = ttk.Entry(mqtt_frame, width=6)
        self.mqtt_port.insert(0, "1883")
        self.mqtt_port.grid(row=0, column=3, padx=5, pady=2)

        ttk.Label(mqtt_frame, text="主题:").grid(row=1, column=0, padx=5, pady=2)
        self.mqtt_topic = ttk.Entry(mqtt_frame)
        self.mqtt_topic.insert(0, "srt1")
        self.mqtt_topic.grid(row=1, column=1, columnspan=3, sticky=tk.EW, padx=5, pady=2)

        # 主SRT流配置
        stream1_frame = ttk.LabelFrame(left_panel, text="主SRT流")
        stream1_frame.pack(fill=tk.X, padx=5, pady=5)

        ttk.Label(stream1_frame, text="地址:").grid(row=0, column=0, padx=5, pady=2)
        self.srt_url1 = ttk.Entry(stream1_frame)
        self.srt_url1.insert(0, "srt://119.23.220.15:8890?streamid=read:ao2car2063135")
        self.srt_url1.grid(row=0, column=1, sticky=tk.EW, padx=5, pady=2)

        self.btn_play1 = ttk.Button(stream1_frame, text="启动主流", 
                                  command=lambda: self.toggle_stream(1))
        self.btn_play1.grid(row=1, column=0, columnspan=2, pady=5)

        # 备SRT流配置
        stream2_frame = ttk.LabelFrame(left_panel, text="备SRT流")
        stream2_frame.pack(fill=tk.X, padx=5, pady=5)

        ttk.Label(stream2_frame, text="地址:").grid(row=0, column=0, padx=5, pady=2)
        self.srt_url2 = ttk.Entry(stream2_frame)
        self.srt_url2.insert(0, "srt://119.23.220.15:8890?streamid=read:live1")
        self.srt_url2.grid(row=0, column=1, sticky=tk.EW, padx=5, pady=2)

        self.btn_play2 = ttk.Button(stream2_frame, text="启动备流", 
                                  command=lambda: self.toggle_stream(2))
        self.btn_play2.grid(row=1, column=0, columnspan=2, pady=5)

        # 状态栏
        self.status_bar = ttk.Label(self.root, relief=tk.SUNKEN)
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)
        self.update_status("就绪")

    def toggle_stream(self, stream_num):
        """切换指定流的播放状态"""
        if stream_num == 1:
            if not self.playing1:
                self.start_stream(1)
            else:
                self.stop_stream(1)
        elif stream_num == 2:
            if not self.playing2:
                self.start_stream(2)
            else:
                self.stop_stream(2)

    def start_stream(self, stream_num):
        """启动指定编号的流"""
        if not self.mqtt_client:
            self.init_mqtt()

        url = self.srt_url1.get() if stream_num == 1 else self.srt_url2.get()
        if stream_num == 1:
            cmd = [
                "ffmpeg",
                "-fflags", "nobuffer",
                "-flags", "low_delay",
                "-flush_packets", "1",
                "-i", url,
                # "-vf", "scale=848:480",
                "-f", "image2pipe",
                "-pix_fmt", "rgb24",
                "-vcodec", "rawvideo",
                # "-threads", "1", 
                "-"
            ]
            self.playing1_stop = False
        else:
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
                url
            ]
        try:
            # 关键修复：备流不需要捕获 stdout，但需捕获 stderr
            if stream_num == 1:
                process = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    bufsize=1024 * 1024 * 8,  # 8MB缓冲区（单位：字节）
                    universal_newlines=False
                )
            else:
                process = subprocess.Popen(
                cmd,
                stderr=subprocess.PIPE
                )
            # 主流通用处理
            if stream_num == 1:
                threading.Thread(
                    target=self.video_decode_thread,
                    args=(process,),
                    daemon=True
                ).start()
                self.process1 = process
                self.playing1 = True
                self.btn_play1.config(text="停止主流")
            else:
                self.process2 = process
                self.playing2 = True
                self.auto_restart2 = True  # 确保自动重启标志已设置
                self.stream2_restart_count = 0  # 重置重启计数器
                self.btn_play2.config(text="停止备流")
                # 启动监控线程（仅备流需要）
                threading.Thread(
                    target=self.monitor_stream2,
                    daemon=True
                ).start()

            self.update_status(f"流{stream_num} 已启动")
        except Exception as e:
            self.update_status(f"流{stream_num} 启动失败: {str(e)}")
            if stream_num == 1:
                self.playing1 = False
            else:
                self.playing2 = False

    def monitor_stream2(self):
        """备流监控线程"""
        while self.playing2 and self.auto_restart2:
            # 检查进程是否存在
            if self.process2 is None:
                break
            
            # 检查进程状态
            return_code = self.process2.poll()
            
            # 状态正常检测（每2秒）
            if return_code is None:
                time.sleep(2)
                continue
                
            # 进程异常退出处理
            error_log = ""
            if self.process2.stderr:
                try:
                    error_log = self.process2.stderr.read().decode('utf-8', errors='ignore')
                except Exception as e:
                    error_log = f"错误日志解码失败: {str(e)}"
            else:
                error_log = "无错误日志输出"
            
            self.update_status(f"备流异常退出，代码: {return_code}\n错误信息: {error_log}")
            
            # 自动重启逻辑
            if self.stream2_restart_count < 10:
                self.stream2_restart_count += 1
                self.root.after(0, self._restart_stream2)
                time.sleep(5)  # 重启间隔
            else:
                self.root.after(0, lambda: self.update_status("备流重启次数已达上限"))
                break

    def _restart_stream2(self):
        """带保护的备流重启"""
        if self.auto_restart2 and self.playing2:
            self.stop_stream(2)
            self.start_stream(2)
            self.update_status(f"备流第{self.stream2_restart_count}次重连...")

    def stop_stream(self, stream_num):
        """停止指定流（改进版）"""
        if stream_num == 1:
            if self.process1:
                self.process1.terminate()
                self.process1 = None
                self.playing1 = False
                self.playing1_stop = True
                self.btn_play1.config(text="启动主流")

        if stream_num == 2:
            if self.process2:
                self.process2.terminate()
                self.auto_restart2 = False  # 阻止自动重启
                try:
                    self.process2.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.process2.kill()
                self.process2 = None
                self.playing2 = False
                self.stream2_restart_count = 0
                self.btn_play2.config(text="启动备流")
                self.update_status(f"流{stream_num} 已停止")

    def init_mqtt(self):
        """初始化MQTT连接"""
        try:
            self.mqtt_client = mqtt.Client()
            self.mqtt_client.connect(
                host=self.mqtt_host.get(),
                port=int(self.mqtt_port.get()),
                keepalive=60
            )
            threading.Thread(target=self.mqtt_client.loop_forever, daemon=True).start()
            self.update_status("MQTT已连接")
        except Exception as e:
            self.update_status(f"MQTT连接失败: {str(e)}")
            self.mqtt_client = None

    def publish_status(self, message):
        """发布状态信息"""
        if self.mqtt_client:
            try:
                self.mqtt_client.publish(
                    topic=self.mqtt_topic.get(),
                    payload=message,
                    qos=1
                )
            except Exception as e:
                self.update_status(f"消息发布失败: {str(e)}")

    def send_message(self):
        """发送自定义消息"""
        message = self.message_entry.get()
        if message:
            self.publish_status(message)
            self.message_entry.delete(0, tk.END)
            self.update_status(f"消息已发送: {message}")
        else:
            messagebox.showwarning("输入错误", "消息内容不能为空")

    def on_enter_pressed(self, event):
        """回车键发送消息"""
        self.send_message()
        return "break"

    def update_status(self, text):
        """更新状态栏"""
        self.status_bar.config(text=text)

    def toggle_bridge(self):
        if self.bridge and self.bridge.running:
            self.stop_bridge()
        else:
            self.start_bridge()

    def start_bridge(self):
        try:
            serial_port = self.serial_port_entry.get()
            baudrate = 420000
            mqtt_server = "119.23.220.15"
            mqtt_port = 1883
            mqtt_topic = "ao2car2063135"

            self.bridge = SerialToMqttBridge(
                serial_port=serial_port,
                baudrate=baudrate,
                mqtt_server=mqtt_server,
                mqtt_port=mqtt_port,
                mqtt_topic=mqtt_topic
            )
            
            self.bridge_thread = threading.Thread(target=self.bridge.run, daemon=True)
            self.bridge_thread.start()
            self.bridge_btn.config(text="停止桥接")
            self.update_status("串口桥接已启动")
        except Exception as e:
            messagebox.showerror("错误", f"启动桥接失败: {str(e)}")
            self.update_status("桥接启动失败")

    def stop_bridge(self):
        if self.bridge:
            self.bridge.stop()
            self.bridge_btn.config(text="启动桥接")
            self.update_status("串口桥接已停止")

    def on_close(self):
        """关闭事件处理"""
        if self.bridge:
            self.bridge.stop()
            self.bridge = None
        if self.process1:
            self.process1.kill() 
            self.process1 = None
        if self.process2:
            self.process2.kill()
            self.process2 = None
        if self.mqtt_client:
            self.mqtt_client.disconnect()
            self.mqtt_client.loop_stop()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = DualSRTPlayerWithMQTT(root)
    # 设置全屏属性（隐藏标题栏）
    root.attributes('-fullscreen', True)  # [1,2,5](@ref)
    # 绑定退出全屏快捷键（如ESC键）
    root.bind("<Escape>", lambda e: root.attributes('-fullscreen', False))
    root.mainloop()

