import paho.mqtt.client as mqtt
import edge_tts
import asyncio
from pydub import AudioSegment
from pydub.playback import play
import io
import threading
import time

# 配置参数
CONFIG = {
    "mqtt_host": "119.23.220.15",
    "mqtt_port": 1883,
    "mqtt_topic": "srt1",
    "voice_name": "zh-CN-YunxiNeural",  # 使用男声音
    "volume_boost": -6,                  # 音量增益(dB)
    "playback_speed": 1.3              # 播放速度
}

class EdgeTTSClient:
    def __init__(self, config):
        self.config = config
        self.loop = asyncio.new_event_loop()

    async def _async_generate(self, text):
        """异步生成语音"""
        communicate = edge_tts.Communicate(
            text,
            self.config['voice_name']
        )
        mp3_data = bytearray()
        async for chunk in communicate.stream():
            if chunk["type"] == "audio":
                mp3_data.extend(chunk["data"])
        return mp3_data

    def text_to_speech(self, text):
        """同步接口生成语音"""
        try:
            mp3_data = self.loop.run_until_complete(self._async_generate(text))

            with io.BytesIO(mp3_data) as fp:
                audio = AudioSegment.from_mp3(fp)
                audio = audio + self.config['volume_boost']
                return audio.speedup(
                    playback_speed=self.config['playback_speed'],
                    chunk_size=100,
                    crossfade=25
                )

        except Exception as e:
            print(f"❌ 语音生成失败: {str(e)}")
            return None

class MQTTSpeechService:
    def __init__(self, config):
        self.config = config
        self.client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
        self.tts = EdgeTTSClient(config)
        self.playback_lock = threading.Lock()

        # 设置回调
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def start(self):
        """启动服务"""
        try:
            self.client.connect(
                self.config['mqtt_host'],
                self.config['mqtt_port'],
                60
            )
            self.client.loop_start()
            print("✅ MQTT连接成功")
            threading.Thread(target=self.keep_alive, daemon=True).start()
        except Exception as e:
            print(f"❌ MQTT连接失败: {str(e)}")
            self.reconnect()

    def on_connect(self, client, userdata, flags, reason_code, properties):
        """连接回调"""
        if reason_code == 0:
            client.subscribe(self.config['mqtt_topic'])
            print(f"👂 已订阅主题: {self.config['mqtt_topic']}")
        else:
            print(f"❌ 连接失败: {mqtt.connack_string(reason_code)}")

    def on_message(self, client, userdata, msg):
        """消息处理回调"""
        try:
            text = msg.payload.decode('utf-8')
            print(f"📩 收到消息: {text}")

            # 启动独立线程处理语音
            threading.Thread(
                target=self.process_message,
                args=(text,),
                daemon=True
            ).start()

        except Exception as e:
            print(f"❌ 消息处理错误: {str(e)}")

    def process_message(self, text):
        """带锁的语音处理"""
        with self.playback_lock:
            if audio := self.tts.text_to_speech(text):
                try:
                    play(audio)
                    print("▶️ 播放完成")
                except Exception as e:
                    print(f"❌ 播放失败: {str(e)}")

    def reconnect(self):
        """重连机制"""
        print("⏳ 5秒后尝试重连...")
        time.sleep(5)
        self.start()

    def keep_alive(self):
        """保持服务运行"""
        while True:
            time.sleep(1)

    def stop(self):
        """停止服务"""
        self.client.disconnect()
        print("🛑 服务已停止")

if __name__ == "__main__":
    service = MQTTSpeechService(CONFIG)
    try:
        print("🚀 启动MQTT语音播报服务...")
        service.start()
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        service.stop()
        print("\n👋 服务已终止")