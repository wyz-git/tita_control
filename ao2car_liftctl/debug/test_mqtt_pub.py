import paho.mqtt.client as mqtt
import time
import random
import json


class MQTT_SERVER:
    def __init__(self):
        print("[Client]: MQTT INIT...")
        self.broker = "broker.emqx.io"
        self.port = 1883
        self.main_topic = "/flask/mqtt"
        self.ack_topic = "/flask/mqtt/ack"
        self.topics = [(self.main_topic, 1), (self.ack_topic, 1)]
        self.client_id = f"python-mqtt-{random.randint(0, 1000)}"
        self.client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id=self.client_id,
        )     

        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(self.broker, self.port)

    def on_connect(self, client, userdata, connect_flags, reason_code, properties):
        if reason_code == 0:
            print("[Debug]: Connected to MQTT Broker!")
        else:
            print(
                "[Debug]: Failed to connect, return code [{}]\r\n".format(reason_code)
            )

    def on_message(self, client, userdata, msg):
        print(msg.topic + " " + str(msg.payload))

    def publish(self, client):
        self.msg_count = 0

        pub_msg = {
            "requestId": "123",
            "heads": {
                "instructionId": "5bf698c8-21b6-4574-b3b2-4ec512d00b0a",
                "thingId": "hhhf689a-f4c4-4055-972b-7d7dddab56e5",
            },
            "body": {"command": "enter"},
        }

        try:
            while True:
                time.sleep(1)
                # self.msg = f"messages: {self.msg_count}"
                result = self.client.publish(self.main_topic, str(pub_msg))
                status = result[0]
                if status == 0:
                    print(f"Send '{pub_msg}' to topic '{self.main_topic}'")
                else:
                    print(f"Failed to send message to topic {self.main_topic}")
                self.msg_count += 1
        except KeyboardInterrupt:
            self.client.loop_stop()

    def run(self):
        self.client.subscribe(self.ack_topic)
        self.client.loop_start()
        self.publish(self.client)


if __name__ == "__main__":
    mqtt_pub = MQTT_SERVER()
    mqtt_pub.run()
