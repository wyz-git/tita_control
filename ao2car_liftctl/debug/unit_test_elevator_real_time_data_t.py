import unittest
from src.struct.elevator_real_time_data_t import ELE_REAL_TIME_DATA_T


class EleRealTimeDataT(unittest.TestCase):
    def setUp(self):
        self.ele_real_time_data = ELE_REAL_TIME_DATA_T("123", "456")

    def test_01_run(self):
        self.ele_real_time_data.CONNECTION_STATE = 1
        print(
            "self.ele_real_time_data.CONNECTION_STATE is [{}]".format(
                self.ele_real_time_data.CONNECTION_STATE
            )
        )
        self.ele_real_time_data.FLOOR = 2
        print(
            "self.ele_real_time_data.FLOOR is [{}]".format(
                self.ele_real_time_data.FLOOR
            )
        )
        self.ele_real_time_data.MOTION = 3
        print(
            "self.ele_real_time_data.MOTION is [{}]".format(
                self.ele_real_time_data.MOTION
            )
        )
        self.ele_real_time_data.LAYER_STATUS = 4
        print(
            "self.ele_real_time_data.LAYER_STATUS is [{}]".format(
                self.ele_real_time_data.LAYER_STATUS
            )
        )
        self.ele_real_time_data.PEOPLE_STATUS = 5
        print(
            "self.ele_real_time_data.PEOPLE_STATUS is [{}]".format(
                self.ele_real_time_data.PEOPLE_STATUS
            )
        )
        self.ele_real_time_data.BACK_DOOR_LIVE = 6
        print(
            "self.ele_real_time_data.BACK_DOOR_LIVE is [{}]".format(
                self.ele_real_time_data.BACK_DOOR_LIVE
            )
        )
        self.ele_real_time_data.FONT_DOOR_LIVE = 7
        print(
            "self.ele_real_time_data.FONT_DOOR_LIVE is [{}]".format(
                self.ele_real_time_data.FONT_DOOR_LIVE
            )
        )
        self.ele_real_time_data.TRACE_ID = 8
        print(
            "self.ele_real_time_data.TRACE_ID is [{}]".format(
                self.ele_real_time_data.TRACE_ID
            )
        )
        self.ele_real_time_data.ELE_STATUS_CODE = 9
        print(
            "self.ele_real_time_data.ELE_STATUS_CODE is [{}]".format(
                self.ele_real_time_data.ELE_STATUS_CODE
            )
        )
        self.ele_real_time_data.PEOPLE_NUMBER = 10
        print(
            "self.ele_real_time_data.PEOPLE_NUMBER is [{}]".format(
                self.ele_real_time_data.PEOPLE_NUMBER
            )
        )
        print("===============")


if __name__ == "__main__":
    unittest.main()
