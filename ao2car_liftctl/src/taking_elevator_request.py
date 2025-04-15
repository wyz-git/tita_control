import requests
import time
import random
import json
from src.tools.log import Log
from src.struct.elevator_real_time_data_t import ELE_REAL_TIME_DATA_T


class TAKING_ELEVATOR_REQUEST:
    def __init__(self):
        print("[Request]: HTTPS INIT...")
        self.Log = Log("TakingElevatorRequest")
        self.return_success = 1
        self.https_response_ok = 200
        self.ele_real_time_data = ELE_REAL_TIME_DATA_T(None, None)

    def init_https_connection(self):
        pass

    def return_config_json_data(self):
        with open("config/https_active_interface.json", "r") as json_file:
            data = json.load(json_file)

        lu_bang_cloud_url_header = data["lu_bang_cloud_url"]["URL_HEADER"]["string"]
        security_key = data["item_config"]["SECURITY_KEY"]["string"]
        elevatir_number = data["elevator_config"]["ELEVATOR_NUMBER"]["string"]

        self.LU_BANG_CLOUD_URL_HEADER = lu_bang_cloud_url_header
        self.SECURITY_KEY = security_key
        self.ELEVATOR_NUMBER = elevatir_number

        self.Log.debug("[Debug]: security_key: [{}]".format(self.SECURITY_KEY))

        return True

    def request_get(self, url, header, payload):
        response = requests.get(url, headers=header, json=payload)
        self.Log.debug(response.text)
        return response

    def request_put(self, url, header, payload):
        response = requests.put(url, headers=header, json=payload)
        self.Log.debug(response.text)
        return response

    def request_post(self, url, header, payload):
        response = requests.post(url, headers=header, json=payload)
        self.Log.debug(response.text)
        return response

    def request_delete(self, url, header, payload):
        response = requests.delete(url, headers=header, json=payload)
        self.Log.debug(response.text)
        return response

    def update_real_time_elevator_info(self, response: requests.Response):
        pass
        # data = response.json()

        # elevator_id = data["result"]["eleRealTimeData"]["elevatorId"]
        # elevator_num = data["result"]["eleRealTimeData"]["elevatorNumber"]
        # connection_state = data["result"]["eleRealTimeData"]["connectionState"]
        # floor = data["result"]["eleRealTimeData"]["floor"]
        # motion = data["result"]["eleRealTimeData"]["motion"]
        # layer_status = data["result"]["eleRealTimeData"]["layerStatus"]
        # people_status = data["result"]["eleRealTimeData"]["peopleStatus"]
        # back_door_live = data["result"]["eleRealTimeData"]["backDoorLive"]
        # font_door_live = data["result"]["eleRealTimeData"]["fontDoorLive"]
        # trace_id = data["result"]["eleRealTimeData"]["traceId"]
        # elevator_status_code = data["result"]["eleRealTimeData"]["elevatorStatusCode"]
        # elevator_status_name = data["result"]["eleRealTimeData"]["elevatorStatusName"]
        # people_num = data["result"]["eleRealTimeData"]["peopleNumber"]

        # if (
        #     self.ele_real_time_data.ELE_ID is None
        #     and self.ele_real_time_data.ELE_NUMBER is None
        # ):
        #     self.Log.error("[Error]: self.ele_real_time_data is None, plz init it!")
        #     return None

        # self.ele_real_time_data = ELE_REAL_TIME_DATA_T(elevator_id, elevator_num)
        # self.ele_real_time_data.CONNECTION_STATE = connection_state
        # self.ele_real_time_data.FLOOR = floor
        # self.ele_real_time_data.MOTION = motion
        # self.ele_real_time_data.LAYER_STATUS = layer_status
        # self.ele_real_time_data.PEOPLE_STATUS = people_status
        # self.ele_real_time_data.BACK_DOOR_LIVE = back_door_live
        # self.ele_real_time_data.FONT_DOOR_LIVE = font_door_live
        # self.ele_real_time_data.TRACE_ID = trace_id
        # self.ele_real_time_data.ELE_STATUS_CODE = elevator_status_code
        # self.ele_real_time_data.ELE_STATUS_NAME = elevator_status_name
        # self.ele_real_time_data.PEOPLE_NUMBER = people_num
        
        # self.Log.info("||||||||||||||||||||||||||||||||")
        # self.Log.info("[Elevator Status]: elevator_id is [{}]".format(self.elevator_id))
        # self.Log.info("[Elevator Status]: connection_state is [{}]".format(self.connection_state))
        # self.Log.info("[Elevator Status]: floor is [{}]".format(self.floor))
        # self.Log.info("[Elevator Status]: motion is [{}]".format(self.motion))
        # self.Log.info("[Elevator Status]: back_door_live is [{}]".format(self.back_door_live))
        # self.Log.info("[Elevator Status]: font_door_live is [{}]".format(self.font_door_live))
        # self.Log.info("[Elevator Status]: elevator_status_code is [{}]".format(self.elevator_status_code))
        # self.Log.info("[Elevator Status]: elevator_status_name is [{}]".format(self.elevator_status_name))
        # self.Log.info("||||||||||||||||||||||||||||||||")
        

    def request_robot_instantiation_identification(self):
        self.ROBOT_CODE = random.randint(1, 1000)
        request_url = (
            self.LU_BANG_CLOUD_URL_HEADER + "/dispatching/robot/" + str(self.ROBOT_CODE)
        )
        self.Log.debug("[Debug]: request_url is [{}]".format(request_url))

        header = {"Content-Type": "application/json", "SECURITY-KEY": self.SECURITY_KEY}

        response = self.request_get(request_url, header, None)
        data = response.json()
        request_result = data["code"]

        if response.status_code != self.https_response_ok:
            self.Log.info(
                "[Info]: https request put return error [{}]".foramt(
                    response.status_code
                )
            )
            return None
        if request_result != str(self.return_success):
            self.Log.info("[Info]: request result is not success...")
            return None

        self.ROBOT_IDENTIFICATION = data.get("result")
        self.Log.info(
            "[Info]: ROBOT_IDENTIFICATION is [{}]".format(self.ROBOT_IDENTIFICATION)
        )

        return response

    def request_enable_robot_instantiation(self):
        request_url = (
            self.LU_BANG_CLOUD_URL_HEADER
            + "/rdms-iot/things/"
            + str(self.ROBOT_IDENTIFICATION)
            + "/activate/lite"
        )
        self.Log.debug("[Debug]: request_url is [{}]".format(request_url))

        header = {"Content-Type": "application/json"}
        payload = {}

        response = self.request_put(request_url, header, payload)
        data = response.json()
        request_result = data["code"]

        if response.status_code != self.https_response_ok:
            self.Log.info(
                "[Info]: https request put return error [{}]".foramt(
                    response.status_code
                )
            )
            return None
        if request_result != str(self.return_success):
            self.Log.info("[Info]: request result is not success...")
            return None

        self.THING_ID = data["result"]["thingId"]
        self.MQTT_CHANNEL_CLIENT_ID = data["result"]["channelData"]["properties"][
            "clientId"
        ]
        self.MQTT_CHANNEL_CLIENT_PASSWORD = data["result"]["channelData"][
            "properties"
        ]["password"]
        self.MQTT_CHANNEL_HOST = data["result"]["channelData"]["properties"]["host"]
        self.MQTT_CHANNEL_TOPIC_SUB_COMMAND = data["result"]["channelData"][
            "properties"
        ]["topic"]["sub"]["command"]
        self.MQTT_CHANNEL_TOPIC_PUB_DATA = data["result"]["channelData"][
            "properties"
        ]["topic"]["pub"]["data"]
        self.MQTT_CHANNEL_TOPIC_PUB_ACK = data["result"]["channelData"][
            "properties"
        ]["topic"]["pub"]["ack"]
        self.MQTT_CHANNEL_USERNAME = data["result"]["channelData"]["properties"][
            "username"
        ]

        self.Log.info("[Info]: self.THING_ID is [{}]".format(self.THING_ID))
        self.Log.info(
            "[Info]: self.MQTT_CHANNEL_CLIENT_ID is [{}]".format(
                self.MQTT_CHANNEL_CLIENT_ID
            )
        )
        self.Log.info(
            "[Info]: self.MQTT_CHANNEL_CLIENT_PASSWORD is [{}]".format(
                self.MQTT_CHANNEL_CLIENT_PASSWORD
            )
        )
        self.Log.info(
            "[Info]: self.MQTT_CHANNEL_HOST is [{}]".format(self.MQTT_CHANNEL_HOST)
        )
        self.Log.info(
            "[Info]: self.MQTT_CHANNEL_TOPIC_SUB_COMMAND is [{}]".format(
                self.MQTT_CHANNEL_TOPIC_SUB_COMMAND
            )
        )
        self.Log.info(
            "[Info]: self.MQTT_CHANNEL_TOPIC_PUB_DATA is [{}]".format(
                self.MQTT_CHANNEL_TOPIC_PUB_DATA
            )
        )
        self.Log.info(
            "[Info]: self.MQTT_CHANNEL_TOPIC_PUB_ACK is [{}]".format(
                self.MQTT_CHANNEL_TOPIC_PUB_ACK
            )
        )
        self.Log.info(
            "[Info]: self.MQTT_CHANNEL_USERNAME is [{}]".format(
                self.MQTT_CHANNEL_USERNAME
            )
        )

        return response

    def request_robot_online(self):
        request_url = (
            self.LU_BANG_CLOUD_URL_HEADER
            + "/rdms-iot/things/"
            + str(self.ROBOT_IDENTIFICATION)
            + "/online"
        )
        self.Log.debug("[Debug]: request_url is [{}]".format(request_url))

        response = self.request_put(request_url, None, None)
        data = response.json()
        request_result = data["code"]

        if response.status_code != self.https_response_ok:
            self.Log.info(
                "[Info]: https request put return error [{}]".foramt(
                    response.status_code
                )
            )
            return None
        if request_result != str(self.return_success):
            self.Log.info("[Info]: request result is not success...")
            return None

        return response

    def request_init_a_ride_request(self, floor_start: int, floor_end: int):
        self.TRACE_ID = "2135557754"
        current_timestamp = time.time()
        current_timestamp_ms = int(current_timestamp * 1000)
        request_url = self.LU_BANG_CLOUD_URL_HEADER + "/dispatching/requests"
        self.Log.debug("[Debug]: request_url is [{}]".format(request_url))

        header = {"Content-Type": "application/json", "SECURITY-KEY": self.SECURITY_KEY}

        payload = {
            "traceId": self.TRACE_ID,
            "passenger": self.ROBOT_IDENTIFICATION,
            "elevatorNumber": self.ELEVATOR_NUMBER,
            "from": floor_start,
            "to": floor_end,
            "time": current_timestamp_ms,
        }

        response = self.request_post(request_url, header, payload)
        data = response.json()
        request_result = data["code"]

        if response.status_code != self.https_response_ok:
            self.Log.info(
                "[Info]: https request put return error [{}]".foramt(
                    response.status_code
                )
            )
            return None
        if request_result != str(self.return_success):
            self.Log.info("[Info]: request result is not success...")
            return None

        elevator_id = data["result"]["eleRealTimeData"]["elevatorId"]
        elevator_num = data["result"]["eleRealTimeData"]["elevatorNumber"]

        self.ELE_ID = elevator_id
        self.ELE_NUMBER = elevator_num

        self.update_real_time_elevator_info(response)

        return response

    def request_complete_enter_the_elevator(self):
        current_timestamp_ms = int(time.time() * 1000)
        request_url = self.LU_BANG_CLOUD_URL_HEADER + "/dispatching/enterFinish"

        header = {"Content-Type": "application/json", "SECURITY-KEY": self.SECURITY_KEY}

        payload = {
            "passenger": self.ROBOT_IDENTIFICATION,
            "traceId": self.TRACE_ID,
            "time": current_timestamp_ms,
        }

        response = self.request_post(request_url, header, payload)
        data = response.json()
        request_result = data["code"]

        if response.status_code != self.https_response_ok:
            self.Log.info(
                "[Info]: https request put return error [{}]".foramt(
                    response.status_code
                )
            )
            return None
        if request_result != str(self.return_success):
            self.Log.info("[Info]: request result is not success...")
            return None

        self.update_real_time_elevator_info(response)

        return response

    def request_complete_exit_the_elevator(self):
        current_timestamp_ms = int(time.time() * 1000)
        request_url = self.LU_BANG_CLOUD_URL_HEADER + "/dispatching/exitFinish"
        self.Log.debug("[Debug]: request_url is [{}]".format(request_url))

        header = {"Content-Type": "application/json", "SECURITY-KEY": self.SECURITY_KEY}

        payload = {
            "passenger": self.ROBOT_IDENTIFICATION,
            "traceld": self.ele_real_time_data.TRACE_ID,
            "time": current_timestamp_ms,
        }

        response = self.request_post(request_url, header, payload)
        data = response.json()
        request_result = data["code"]

        if response.status_code != self.https_response_ok:
            self.Log.info(
                "[Info]: https request put return error [{}]".foramt(
                    response.status_code
                )
            )
            return None
        if request_result != str(self.return_success):
            self.Log.info("[Info]: request result is not success...")
            return None

        self.update_real_time_elevator_info(response)

        return response

    def request_cancel_elevator_work(self):
        current_timestamp_ms = int(time.time() * 1000)
        request_url = self.LU_BANG_CLOUD_URL_HEADER + "/dispatching/cancel"
        
        self.Log.debug("current_timestamp_ms is [{}]".format(current_timestamp_ms))

        header = {"Content-Type": "application/json", "SECURITY-KEY": self.SECURITY_KEY}

        payload = {
            "passenger": self.ROBOT_IDENTIFICATION,
            "traceId": self.TRACE_ID,
            "time": current_timestamp_ms,
            "cause": "active cancel by admin",
        }

        response = self.request_delete(request_url, header, payload)
        data = response.json()
        request_result = data["code"]

        if response.status_code != self.https_response_ok:
            self.Log.info(
                "[Info]: https request put return error [{}]".foramt(
                    response.status_code
                )
            )
            return None
        if request_result != str(self.return_success):
            self.Log.info("[Info]: request result is not success...")
            return None

        # self.update_real_time_elevator_info(response)
                    
        return response
        
    def request_elevator_condiftion(self):
        request_url = (
            self.LU_BANG_CLOUD_URL_HEADER
            + "/dispatching/eleStatus/"
            + str(self.ROBOT_IDENTIFICATION)
        )

        self.Log.debug("[request_elevator_condiftion] Request url is [{}]".format(request_url))

        header = {"Content-Type": "application/json", "SECURITY-KEY": self.SECURITY_KEY}

        response = self.request_get(request_url, header, None)
        data = response.json()
        request_result = data["code"]

        if response.status_code != self.https_response_ok:
            self.Log.info(
                "[Info]: https request put return error [{}]".foramt(
                    response.status_code
                )
            )
            return None
        if request_result != str(self.return_success):
            self.Log.info("[Info]: request result is not success...")
            return None

        elevator_id = data["result"]["eleRealTimeDataList"][0]["elevatorId"]
        elevator_num = data["result"]["eleRealTimeDataList"][0]["elevatorNumber"]
        connection_state = data["result"]["eleRealTimeDataList"][0]["connectionState"]
        floor = data["result"]["eleRealTimeDataList"][0]["floor"]
        motion = data["result"]["eleRealTimeDataList"][0]["motion"]
        layer_status = data["result"]["eleRealTimeDataList"][0]["layerStatus"]
        people_status = data["result"]["eleRealTimeDataList"][0]["peopleStatus"]
        back_door_live = data["result"]["eleRealTimeDataList"][0]["backDoorLive"]
        font_door_live = data["result"]["eleRealTimeDataList"][0]["fontDoorLive"]
        trace_id = data["result"]["eleRealTimeDataList"][0]["traceId"]
        elevator_status_code = data["result"]["eleRealTimeDataList"][0]["elevatorStatusCode"]
        elevator_status_name = data["result"]["eleRealTimeDataList"][0]["elevatorStatusName"]
        people_num = data["result"]["eleRealTimeDataList"][0]["peopleNumber"]            
        
        self._CONNECTION_STATE = connection_state
        self._FLOOR = floor
        self._MOTION = motion
        self._FONT_DOOR_LIVE = font_door_live
        
        self.Log.info("||||||||||||||||||||||||||||||||")
        self.Log.info("[Elevator Status]: elevator_id is [{}]".format(elevator_id))
        self.Log.info("[Elevator Status]: connection_state is [{}]".format(connection_state))
        self.Log.info("[Elevator Status]: floor is [{}]".format(floor))
        self.Log.info("[Elevator Status]: motion is [{}]".format(motion))
        self.Log.info("[Elevator Status]: back_door_live is [{}]".format(back_door_live))
        self.Log.info("[Elevator Status]: font_door_live is [{}]".format(font_door_live))
        self.Log.info("[Elevator Status]: elevator_status_code is [{}]".format(elevator_status_code))
        self.Log.info("[Elevator Status]: elevator_status_name is [{}]".format(elevator_status_name))
        self.Log.info("||||||||||||||||||||||||||||||||")

        return response

    def request_for_exit_elevator(self, floor_end: int):
        current_timestamp = time.time()
        current_timestamp_ms = int(current_timestamp * 1000)
        request_url = (
            self.LU_BANG_CLOUD_URL_HEADER
            + "/dispatching/eleStatus/"
            + str(self.ROBOT_IDENTIFICATION)
        )

        header = {"Content-Type": "application/json", "SECURITY-KEY": self.SECURITY_KEY}

        payload = {
            "traceId": self.ele_real_time_data.TRACE_ID,
            "passenger": self.ROBOT_IDENTIFICATION,
            "to": floor_end,
            "elevatorNumber": self.ELE_NUMBER,
            "time": current_timestamp_ms,
        }

        response = self.request_post(request_url, header, payload)
        data = response.json()
        request_result = data["code"]

        if response.status_code != self.https_response_ok:
            self.Log.info(
                "[Info]: https request put return error [{}]".foramt(
                    response.status_code
                )
            )
            return None
        if request_result != str(self.return_success):
            self.Log.info("[Info]: request result is not success...")
            return None

        self.update_real_time_elevator_info(response)
        return response

    def request_elevator_delay_open_the_door(self):
        request_url = self.LU_BANG_CLOUD_URL_HEADER + "/dispatching/openDoor"

        header = {"Content-Type": "application/json", "SECURITY-KEY": self.SECURITY_KEY}

        payload = {
            "passenger": self.ROBOT_IDENTIFICATION,
            "elevatorNumber": self.ELE_NUMBER,
        }

        response = self.request_post(request_url, header, None)
        data = response.json()
        request_result = data["code"]

        if response.status_code != self.https_response_ok:
            self.Log.info(
                "[Info]: https request put return error [{}]".foramt(
                    response.status_code
                )
            )
            return None
        if request_result != str(self.return_success):
            self.Log.info("[Info]: request result is not success...")
            return None

        return response

    def return_MQTT_CHANNEL_TOPIC_SUB_COMMAND(self):
        return self.MQTT_CHANNEL_TOPIC_SUB_COMMAND
    
    def return_MQTT_CHANNEL_TOPIC_PUB_DATA(self):
        return self.MQTT_CHANNEL_TOPIC_PUB_DATA
    
    def return_MQTT_CHANNEL_TOPIC_PUB_ACK(self):
        return self.MQTT_CHANNEL_TOPIC_PUB_ACK
    
    def return_MQTT_CHANNEL_CLIENT_ID(self):
        return self.MQTT_CHANNEL_CLIENT_ID
    
    def return_MQTT_CHANNEL_CLIENT_PASSWORD(self):
        return self.MQTT_CHANNEL_CLIENT_PASSWORD
    
    def return_MQTT_CHANNEL_USERNAME(self):
        return self.MQTT_CHANNEL_USERNAME

    def return_MQTT_CHANNEL_HOST(self):
        return self.MQTT_CHANNEL_HOST
    
    def return_CONNECTION_STATE(self):
        return self._CONNECTION_STATE

    def return_FLOOR(self):
        return self._FLOOR
    
    def return_MOTION(self):
        return self._MOTION
    
    def return_FONT_DOOR_LIVE(self):
        return self._FONT_DOOR_LIVE
    