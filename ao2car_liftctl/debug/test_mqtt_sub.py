import paho.mqtt.client as mqtt
import time
import random
import ast


class MQTT_CLIENT:
    def __init__(self):
        print("[Client]: MQTT INIT...")
        self.broker = "broker.emqx.io"
        self.port = 1883
        self.main_topic = "/flask/mqtt"
        self.ack_topic = "/flask/mqtt/ack"
        self.topics = [(self.main_topic, 1), (self.ack_topic, 1)]
        self.client_id = f"python-mqtt-{random.randint(0, 100)}"
        self.client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id=self.client_id,
        )
        self.client.on_connect = self.on_connectC
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
        response = msg.payload.decode("utf-8")
        data = ast.literal_eval(response)

        print("============== data is [{}]".format(data))

        try:
            if "requestId" not in data:
                return

            if msg.topic == self.main_topic:
                request_id = data["requestId"]
                get_ins_id = data["heads"]["instructionId"]
                get_thing_id = data["heads"]["thingId"]
                get_command = data["body"]["command"]

                # print("request_id is [{}]".format(request_id))
                # print("get_ins_id is [{}]".format(get_ins_id))
                # print("get_thing_id is [{}]".format(get_thing_id))
                # print("get_command is [{}]".format(get_command))

                ack_msg = {
                    "requestId": request_id,
                    "heads": {
                        "instructionId": get_ins_id,
                        "thingId": get_thing_id,
                    },
                    "body": {"statusCode": "1", "status": "Success"},
                }

                self.client.publish(self.ack_topic, str(ack_msg))

            elif msg.topic == self.ack_topic:
                print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")
        except Exception as e:
            print("[Error]: something error ...")

    def subscribe(self, client):
        self.client.subscribe(self.topics)

    def run(self):
        self.subscribe(self.client)
        self.client.loop_forever()


if __name__ == "__main__":
    mqtt_client = MQTT_CLIENT()
    mqtt_client.run()
