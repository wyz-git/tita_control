import unittest
from src.taking_elevator_request import TAKING_ELEVATOR_REQUEST
from src.update_elevator_status import ELEVATOR_COMMAND_GET
import time
import threading

class TestTakingElevatorRequest(unittest.TestCase):
    def setUp(self):
        self.taking_elevator_request = TAKING_ELEVATOR_REQUEST()
        self.elevator_command_get = ELEVATOR_COMMAND_GET()

    def test_01_run(self):
        self.taking_elevator_request.return_config_json_data()
        print("===============")

    def test_02_run(self):
        self.taking_elevator_request.return_config_json_data()
        self.taking_elevator_request.request_robot_instantiation_identification()
        time.sleep(1)
        self.taking_elevator_request.request_enable_robot_instantiation()
        time.sleep(1)
        # self.taking_elevator_request.request_robot_online()
        # time.sleep(1)
        # self.taking_elevator_request.request_init_a_ride_request(8, 9)
        # time.sleep(1)
        self.elevator_command_get.get_MQTT_CHANNEL_TOPIC_SUB_COMMAND(self.taking_elevator_request.return_MQTT_CHANNEL_TOPIC_SUB_COMMAND())
        self.elevator_command_get.get_MQTT_CHANNEL_TOPIC_PUB_DATA(self.taking_elevator_request.return_MQTT_CHANNEL_TOPIC_PUB_DATA())
        self.elevator_command_get.get_MQTT_CHANNEL_TOPIC_PUB_ACK(self.taking_elevator_request.return_MQTT_CHANNEL_TOPIC_PUB_ACK())
        self.elevator_command_get.get_MQTT_CHANNEL_CLIENT_ID(self.taking_elevator_request.return_MQTT_CHANNEL_CLIENT_ID())
        self.elevator_command_get.get_MQTT_CHANNEL_CLIENT_PASSWORD(self.taking_elevator_request.return_MQTT_CHANNEL_CLIENT_PASSWORD())
        self.elevator_command_get.get_MQTT_CHANNEL_HOST(self.taking_elevator_request.return_MQTT_CHANNEL_HOST())
        self.elevator_command_get.get_MQTT_CHANNEL_USERNAME(self.taking_elevator_request.return_MQTT_CHANNEL_USERNAME())
        self.elevator_command_get.connect()
        self.elevator_command_get.run()
        
        # self.taking_elevator_request.request_cancel_elevator_work()
        time.sleep(1)
        
        print("===============")

    def test_03_run(self):
        print("===============")
        pass


if __name__ == "__main__":
    unittest.main()
