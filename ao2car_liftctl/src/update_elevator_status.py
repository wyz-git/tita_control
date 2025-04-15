import paho.mqtt.client as mqtt
import time
import random
import json
import _thread
import ast
from src.tools.log import Log


class ELEVATOR_COMMAND_GET:
    def __init__(self):
        self.Log = Log("ElevatorStatusGet")
        self.Log.info("[ELEVATOR_COMMAND_GET]: MQTT INIT...")

    def on_connect(self, client, userdata, connect_flags, reason_code, properties):
        if reason_code == 0:
            self.Log.info("[Debug]: Success Connected to MQTT Broker!")
        else:
            self.Log.info(
                "[Debug]: Failed to connect, return code [{}]\r\n".format(reason_code)
            )

    def on_message(self, client, userdata, msg):
        self.Log.info("test...")
        self.Log.info(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")
        response = msg.payload.decode("utf-8")
        data = ast.literal_eval(response)

        if "requestId" not in data:
            return

        request_id = data["requestId"]
        INSTRUCTION_ID = data["heads"]["instructionId"]
        THING_ID = data["heads"]["thingId"]
        COMMAND = data["body"]["command"]

        ## return command ACK
        if COMMAND == "enter" or COMMAND == "leave" or COMMAND == "cancel" or COMMAND == "unableEnter" or COMMAND == "unableLeave":
            ##! The device is successfully received by default. Modify it if necessary
            ack_msg = {
                "requestId": request_id,
                "heads": {
                    "instructionId": INSTRUCTION_ID,
                    "thingId": THING_ID,
                },
                "body": {"statusCode": "1", "status": "Success"},
            }

            self.client.publish(self.ack_topic, str(ack_msg))
        elif COMMAND == "eleStatusData":
            self.ELE_ID = data["body"]["eleStatusData"]["realTimeData"][
                "elevatorId"
            ]
            self.ELE_NUM = data["body"]["eleStatusData"]["realTimeData"][
                "elevatorNumber"
            ]
            self.ELE_CON_STATE = data["body"]["eleStatusData"]["realTimeData"][
                "connectionState"
            ]
            self.ELE_FLOOR = data["body"]["eleStatusData"]["realTimeData"]["floor"]
            self.ELE_MOTION = data["body"]["eleStatusData"]["realTimeData"][
                "motion"
            ]
            self.ELE_LAYER_STATUS = data["body"]["eleStatusData"]["realTimeData"][
                "layerStatus"
            ]
            self.ELE_PEOPLE_STATUS = data["body"]["eleStatusData"]["realTimeData"][
                "peopleStatus"
            ]
            self.ELE_BACK_DOOR_LIVE = data["body"]["eleStatusData"]["realTimeData"][
                "backDoorLive"
            ]
            self.ELE_FONT_DOOR_LIVE = data["body"]["eleStatusData"]["realTimeData"][
                "fontDoorLive"
            ]
            self.TASK_ID = data["body"]["eleStatusData"]["taskData"]["taskId"]
            self.TASK_TRACE_ID = data["body"]["eleStatusData"]["taskData"][
                "traceId"
            ]
            self.TASK_ROBOT_ID = data["body"]["eleStatusData"]["taskData"][
                "robotId"
            ]
            self.TASK_ROBOT_CODE = data["body"]["eleStatusData"]["taskData"][
                "robotCode"
            ]
            self.TASK_ELE_ID = data["body"]["eleStatusData"]["taskData"][
                "elevatorId"
            ]
            self.TASK_DEPARTURE_FLOOR = data["body"]["eleStatusData"]["taskData"][
                "departureFloor"
            ]
            self.TASK_DESTINATION_FLOOR = data["body"]["eleStatusData"]["taskData"][
                "destinationFloor"
            ]
            self.TASK_STATUS_CODE = data["body"]["eleStatusData"]["taskData"][
                "taskStatusCode"
            ]
            self.TASK_STATUS_NAME = data["body"]["eleStatusData"]["taskData"][
                "taskStatusName"
            ]

            self.Log.info("[Elevator Status] TASK_ELE_ID is [{}].".format(self.TASK_ELE_ID))
            self.Log.info("[Elevator Status] ELE_CON_STATE is [{}].".format(self.ELE_CON_STATE))
            self.Log.info("[Elevator Status] COMMAND is [{}].".format(self.COMMAND))
            self.Log.info("[Elevator Status] ELE_FLOOR is [{}].".format(self.ELE_FLOOR))
            self.Log.info("[Elevator Status] ELE_MOTION is [{}].".format(self.ELE_MOTION))
            self.Log.info("[Elevator Status] ELE_BACK_DOOR_LIVE is [{}].".format(self.ELE_BACK_DOOR_LIVE))
            self.Log.info("[Elevator Status] ELE_FONT_DOOR_LIVE is [{}].".format(self.ELE_FONT_DOOR_LIVE))
            
            ##! The device is successfully received by default. Modify it if necessary
            ack_msg = {
                "requestId": request_id,
                "heads": {
                    "instructionId": INSTRUCTION_ID,
                    "thingId": THING_ID,
                },
                "body": {"statusCode": "1", "status": "Success"},
            }

            self.client.publish(self.data_topic, str(ack_msg))

    def subscribe(self, client):
        self.topics = str(self.command_topic)
        # self.topics = str(self.data_topic)
        # self.topics = str(self.ack_topic)
        self.Log.info(f"Data topic: {self.data_topic}")
        self.Log.info(f"Command topic: {self.command_topic}")
        self.Log.info(f"ACK topic: {self.ack_topic}")
        self.Log.info(f"Subscribe to MQTT Broker: {self.topics}")
        self.client.subscribe(self.topics)

    def connect(self):
        self.Log.info("[INFO]: cloud_broker is [{}]".format(self.clould_broker))
        self.Log.info("[INFO]: cloud_port is [{}]".format(self.cloud_port))
        self.Log.info("[INFO]: client_id is [{}]".format(self.client_id))
        self.Log.info("[INFO]: client_password is [{}]".format(self.client_password))

        self.client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id=self.client_id,
        )

        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.username_pw_set(self.username, self.client_password)
        self.client.connect(self.clould_broker, self.cloud_port)

    def run(self):
        self.subscribe(self.client)
        self.client.loop_forever()

    def get_MQTT_CHANNEL_TOPIC_SUB_COMMAND(self, MQTT_CHANNEL_TOPIC_SUB_COMMAND):
        self.command_topic = MQTT_CHANNEL_TOPIC_SUB_COMMAND
    
    def get_MQTT_CHANNEL_TOPIC_PUB_DATA(self, MQTT_CHANNEL_TOPIC_PUB_DATA):
        self.data_topic = MQTT_CHANNEL_TOPIC_PUB_DATA
    
    def get_MQTT_CHANNEL_TOPIC_PUB_ACK(self, MQTT_CHANNEL_TOPIC_PUB_ACK):
        self.ack_topic = MQTT_CHANNEL_TOPIC_PUB_ACK

    def get_MQTT_CHANNEL_CLIENT_ID(self, MQTT_CHANNEL_CLIENT_ID):
        self.client_id = MQTT_CHANNEL_CLIENT_ID
        self.Log.info("[INFO]: MQTT_CHANNEL_CLIENT_ID is [{}]".format(self.client_id))
    
    def get_MQTT_CHANNEL_CLIENT_PASSWORD(self, MQTT_CHANNEL_CLIENT_PASSWORD):
        self.client_password = MQTT_CHANNEL_CLIENT_PASSWORD
        self.Log.info("[INFO]: MQTT_CHANNEL_CLIENT_PASSWORD is [{}]".format(self.client_password))
    
    def get_MQTT_CHANNEL_USERNAME(self, MQTT_CHANNEL_USERNAME):
        self.username = MQTT_CHANNEL_USERNAME
        self.Log.info("[INFO]: MQTT_CHANNEL_USERNAME is [{}]".format(self.username))

    def get_MQTT_CHANNEL_HOST(self, MQTT_CHANNEL_HOST):
        temp_clould_broker = MQTT_CHANNEL_HOST
        if temp_clould_broker.startswith("tcp://"):
            temp_clould_broker = temp_clould_broker[6:]  # 去掉前六个字符
        protocol, address_port = temp_clould_broker.rsplit(":", 1)
        self.clould_broker = protocol
        self.cloud_port = int(address_port)
        self.Log.info("MQTT_CHANNEL_HOST: [{} |and| {}]".format(self.clould_broker, self.cloud_port))

