import requests
import time
import random
import json
from src.tools.log import Log

class DoorControl():
        def __init__(self):
                print("[Request]: HTTPS INIT...")
                self.Log = Log("DoorControl")

        def request_post(self, url, header, payload):
                response = requests.post(url, headers=header, json=payload)
                self.Log.debug(response.text)
                return response

        def door_control(self):
                self.ROBOT_CODE = random.randint(1, 1000)
                request_url = (
                "http://192.168.10.57:48208/opendoor"
                )
                self.Log.debug("[Debug]: request_url is [{}]".format(request_url))

                header = {"Authorization": "Bearer cJCILPiN3KQJkM$3"}

                payload = {
                        "user_id":"ROBOT", 
                        "message":"robot_opendoor"
                }

                response = self.request_post(request_url, header, payload)
                # data = response.json()
                return response

if __name__ == "__main__":
        main = Main()
        main.door_control()