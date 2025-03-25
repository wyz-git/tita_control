import tkinter as tk
from tkinter import ttk, messagebox
import subprocess
import threading
import paho.mqtt.client as mqtt

class DualSRTPlayerWithMQTT:
    def __init__(self, root):
        self.root = root
        self.root.title("双路SRT流播放器 + MQTT发布端")

        # 初始化变量
        self.process1 = None  # 主SRT流进程
        self.process2 = None  # 备SRT流进程
        self.mqtt_client = None
        self.playing1 = False  # 主流状态
        self.playing2 = False  # 备流状态

        # 配置界面布局
        self.create_widgets()

        # 窗口关闭事件
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def create_widgets(self):
        # 主容器配置
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True)

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
        self.srt_url1.insert(0, "srt://119.23.220.15:8890?streamid=read:live")
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

        # 消息发送区域
        msg_frame = ttk.LabelFrame(left_panel, text="消息发布")
        msg_frame.pack(fill=tk.X, padx=5, pady=5)

        self.message_entry = ttk.Entry(msg_frame)
        self.message_entry.pack(fill=tk.X, padx=5, pady=2)
        self.message_entry.bind("<Return>", self.on_enter_pressed)

        self.btn_publish = ttk.Button(msg_frame, text="发送消息", 
                                    command=self.send_message)
        self.btn_publish.pack(pady=5)

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
                "ffplay",
                "-x", "960",
                "-y", "600",
                "-fflags", "nobuffer",
                "-flags", "low_delay",
                "-analyzeduration", "0",
                "-sync", "ext",
                "-f", "mpegts",
                url
            ]
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
            if stream_num == 1:
                self.process1 = subprocess.Popen(cmd)
                self.playing1 = True
                self.btn_play1.config(text="停止主流")
            else:
                self.process2 = subprocess.Popen(cmd)
                self.playing2 = True
                self.btn_play2.config(text="停止备流")

            #self.publish_status(f"stream{stream_num}/started")
            self.update_status(f"流{stream_num} 已启动")
        except Exception as e:
            self.update_status(f"流{stream_num} 启动失败: {str(e)}")
            if stream_num == 1:
                self.playing1 = False
            else:
                self.playing2 = False

    def stop_stream(self, stream_num):
        """停止指定编号的流"""
        process = self.process1 if stream_num == 1 else self.process2
        if process:
            process.terminate()
            if stream_num == 1:
                self.process1 = None
                self.playing1 = False
                self.btn_play1.config(text="启动主流")
            else:
                self.process2 = None
                self.playing2 = False
                self.btn_play2.config(text="启动备流")

            #self.publish_status(f"stream{stream_num}/stopped")
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

    def on_close(self):
        """关闭事件处理"""
        if self.process1:
            self.process1.terminate()
        if self.process2:
            self.process2.terminate()
        if self.mqtt_client:
            self.mqtt_client.disconnect()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = DualSRTPlayerWithMQTT(root)
    root.geometry("350x400")
    root.mainloop()
