from src.taking_elevator_request import TAKING_ELEVATOR_REQUEST
from src.update_elevator_status import ELEVATOR_COMMAND_GET
from src.tools.log import Log
import time
import threading

class FllorControl():
        def __init__(self,gui_instance,start_floor=1, exit_floor=9):
                self.Log = Log("main")
                self.wait_for_request = True
                self.start_enter_elevator_is_ready = False
                self.start_exit_elevator_is_ready = False
                self.start_floor = start_floor
                self.exit_floor = exit_floor
                self.gui = gui_instance
                self.taking_elevator_request = TAKING_ELEVATOR_REQUEST()
                self.elevator_command_get = ELEVATOR_COMMAND_GET() 
                self.thread_1 = threading.Thread(target=self.request_for_elevator_order)
                self.thread_2 = threading.Thread(target=self.get_elevaotor_command)
                self.get_elevator_status_timer = threading.Timer(1, self.get_elevator_status)
                self.cancel_take_elevator_timer = threading.Timer(1, self.cancel_take_elevator)
                
        def test_run(self):
                self.cancel_take_elevator_timer.start()
                self.get_elevator_status_timer.start()
                self.thread_2.start()
                self.thread_1.start()
        
        def cancel_take_elevator(self):
                while self.wait_for_request:
                        self.Log.info("waitting for request")
                        time.sleep(2)
                time.sleep(90)
                self.Log.info("cancel take the elevator")
                self.taking_elevator_request.request_cancel_elevator_work()
                

        def get_elevator_status(self):
                while self.wait_for_request:
                        self.Log.info("waitting for request")
                        time.sleep(1)
                
                try:
                        self.Log.info("get the elevator status")
                        while True:
                                self.taking_elevator_request.request_elevator_condiftion()
                                CONNECTION_STATE = self.taking_elevator_request.return_CONNECTION_STATE()
                                FLOOR = self.taking_elevator_request.return_FLOOR()
                                MOTION = self.taking_elevator_request.return_MOTION()
                                FONT_DOOR_LIVE = self.taking_elevator_request.return_FONT_DOOR_LIVE()
                                
                                self.Log.info("------------------------------")
                                self.Log.info("self.start_enter_elevator_is_ready: [{}]".format(self.start_enter_elevator_is_ready))
                                self.Log.info("self.start_exit_elevator_is_ready: [{}]".format(self.start_exit_elevator_is_ready))
                                self.Log.info("CONNECTION_STATE: [{}]".format(CONNECTION_STATE))
                                self.Log.info("FLOOR: [{}]".format(FLOOR))
                                self.Log.info("MOTION: [{}]".format(MOTION))
                                self.Log.info("FONT_DOOR_LIVE: [{}]".format(FONT_DOOR_LIVE))
                                self.Log.info("------------------------------")
                                # 转换显示格式
                                door_status = "开启" if FONT_DOOR_LIVE == 4 else "关闭"
                                motion_map = {
                                "STOP": "待机", 
                                "UP": "上行▲", 
                                "DOWN": "下行▼"
                                }
                                self.gui.root.after(0, lambda: [
                                self.gui.floor_var.set(f"{FLOOR}F"),
                                self.gui.motion_var.set(motion_map.get(MOTION, "异常")),
                                self.gui.door_var.set(door_status)
                                ])
                                time.sleep(2)                                        
                                if self.start_enter_elevator_is_ready == False and self.start_exit_elevator_is_ready == False:
                                        self.Log.info("..........enter 1..........")
                                        if CONNECTION_STATE == "ONLINE" and FLOOR == self.start_floor and MOTION == "STOP" and FONT_DOOR_LIVE == 4:
                                                self.start_enter_elevator_is_ready = True
                                                self.Log.info("================> ready to enter elevotar <=================")
                                        else:
                                                self.Log.info(".......... no enter if 1 ............")
                                elif self.start_enter_elevator_is_ready == True and self.start_exit_elevator_is_ready == False:
                                        self.Log.info("..........enter 2..........")
                                        if CONNECTION_STATE == "ONLINE" and FLOOR == self.exit_floor and MOTION == "STOP" and FONT_DOOR_LIVE == 4:
                                                self.start_exit_elevator_is_ready = True
                                                self.Log.info("================> ready to exit elevotar <=================")   
                                        else:
                                                self.Log.info(".......... no enter if 2 ............")                               
                                
                except Exception as e:
                        self.Log.error("timer task error [{}]".format(e))
                        
        def request_for_elevator_order(self):
                self.Log.info("start request_for_elevator_order")
                self.taking_elevator_request.return_config_json_data()
                ## 3.1
                response = self.taking_elevator_request.request_robot_instantiation_identification()
                time.sleep(1)
                ## 3.2
                self.taking_elevator_request.request_enable_robot_instantiation()
                time.sleep(1)
                ## 3.3
                self.taking_elevator_request.request_robot_online()
                time.sleep(1)
                ## 4.1
                self.taking_elevator_request.request_init_a_ride_request(self.start_floor, self.exit_floor)
                time.sleep(1)
                if response != None:
                        self.wait_for_request = False
                        
                self.Log.info("wait for start_enter_elevator_is_ready")
                        
                while self.start_enter_elevator_is_ready == False:
                        time.sleep(1)
                            
                ## 4.7
                self.taking_elevator_request.request_elevator_delay_open_the_door()
                time.sleep(5)
                                 
                self.taking_elevator_request.request_complete_enter_the_elevator()
                self.Log.info(">>>>>>>>>> enter the elevator <<<<<<<<<<<<")
                time.sleep(1)
                
                self.Log.info("wait for start_exit_elevator_is_ready")
                
                while self.start_exit_elevator_is_ready == False:
                        time.sleep(1)

                ## 4.7
                self.taking_elevator_request.request_elevator_delay_open_the_door()
                time.sleep(5)
      
                self.taking_elevator_request.request_complete_exit_the_elevator()
                self.Log.info(">>>>>>>>>> exit the elevator <<<<<<<<<<<<")
                time.sleep(1)
        
        def get_elevaotor_command(self):
                while self.wait_for_request:
                        self.Log.info("waitting for request")
                        time.sleep(1)
                
                # try:
                self.Log.info("start get_elevaotor_command")
                self.elevator_command_get.get_MQTT_CHANNEL_TOPIC_SUB_COMMAND(self.taking_elevator_request.return_MQTT_CHANNEL_TOPIC_SUB_COMMAND())
                self.elevator_command_get.get_MQTT_CHANNEL_TOPIC_PUB_DATA(self.taking_elevator_request.return_MQTT_CHANNEL_TOPIC_PUB_DATA())
                self.elevator_command_get.get_MQTT_CHANNEL_TOPIC_PUB_ACK(self.taking_elevator_request.return_MQTT_CHANNEL_TOPIC_PUB_ACK())
                self.elevator_command_get.get_MQTT_CHANNEL_CLIENT_ID(self.taking_elevator_request.return_MQTT_CHANNEL_CLIENT_ID())
                self.elevator_command_get.get_MQTT_CHANNEL_CLIENT_PASSWORD(self.taking_elevator_request.return_MQTT_CHANNEL_CLIENT_PASSWORD())
                self.elevator_command_get.get_MQTT_CHANNEL_HOST(self.taking_elevator_request.return_MQTT_CHANNEL_HOST())
                self.elevator_command_get.get_MQTT_CHANNEL_USERNAME(self.taking_elevator_request.return_MQTT_CHANNEL_USERNAME())
                self.elevator_command_get.connect()
                self.elevator_command_get.run()
                # except Exception as e:
                #         self.Log.error("get elevator command task error [{}]".format(e))
                
if __name__ == '__main__':
        main = Main()
        main.test_run()
                
