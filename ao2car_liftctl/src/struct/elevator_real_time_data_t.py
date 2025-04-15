class ELE_REAL_TIME_DATA_T:
    def __init__(self, ele_id, ele_num):
        self._ELE_ID = ele_id
        self._ELE_NUMBER = ele_num
        self._CONNECTION_STATE = None
        self._FLOOR = None
        self._MOTION = None
        self._LAYER_STATUS = None
        self._PEOPLE_STATUS = None
        self._BACK_DOOR_LIVE = None
        self._FONT_DOOR_LIVE = None
        self._TRACE_ID = None
        self._ELE_STATUS_CODE = None
        self._ELE_STATUS_NAME = None
        self._PEOPLE_NUMBER = None

    @property
    def ELE_ID(self):
        return self._ELE_ID

    @ELE_ID.setter
    def ELE_ID(self, ele_id):
        self._ELE_ID = ele_id

    @property
    def ELE_NUMBER(self):
        return self._ELE_NUMBER

    @ELE_NUMBER.setter
    def ELE_NUMBER(self, ele_num):
        self._ELE_NUMBER = ele_num

    @property
    def CONNECTION_STATE(self):
        return self._CONNECTION_STATE

    @CONNECTION_STATE.setter
    def CONNECTION_STATE(self, con_state):
        self._CONNECTION_STATE = con_state

    @property
    def FLOOR(self):
        return self._FLOOR

    @FLOOR.setter
    def FLOOR(self, floor):
        self._FLOOR = floor

    @property
    def MOTION(self):
        return self._MOTION

    @MOTION.setter
    def MOTION(self, motion):
        self._MOTION = motion

    @property
    def LAYER_STATUS(self):
        return self._LAYER_STATUS

    @LAYER_STATUS.setter
    def LAYER_STATUS(self, layer_status):
        self._LAYER_STATUS = layer_status

    @property
    def PEOPLE_STATUS(self):
        return self._PEOPLE_STATUS

    @PEOPLE_STATUS.setter
    def PEOPLE_STATUS(self, people_status):
        self._PEOPLE_STATUS = people_status

    @property
    def BACK_DOOR_LIVE(self):
        return self._BACK_DOOR_LIVE

    @BACK_DOOR_LIVE.setter
    def BACK_DOOR_LIVE(self, back_door_live):
        self._BACK_DOOR_LIVE = back_door_live

    @property
    def FONT_DOOR_LIVE(self):
        return self._FONT_DOOR_LIVE

    @FONT_DOOR_LIVE.setter
    def FONT_DOOR_LIVE(self, font_door_live):
        self._FONT_DOOR_LIVE = font_door_live

    @property
    def TRACE_ID(self):
        return self._TRACE_ID

    @TRACE_ID.setter
    def TRACE_ID(self, trace_id):
        self._TRACE_ID = trace_id

    @property
    def ELE_STATUS_CODE(self):
        return self._ELE_STATUS_CODE

    @ELE_STATUS_CODE.setter
    def ELE_STATUS_CODE(self, ele_status_code):
        self._ELE_STATUS_CODE = ele_status_code

    @property
    def ELE_STATUS_NAME(self):
        return self._ELE_STATUS_CODE

    @ELE_STATUS_NAME.setter
    def ELE_STATUS_NAME(self, ele_status_name):
        self._ELE_STATUS_NAME = ele_status_name

    @property
    def PEOPLE_NUMBER(self):
        return self._PEOPLE_NUMBER

    @PEOPLE_NUMBER.setter
    def PEOPLE_NUMBER(self, people_num):
        self._PEOPLE_NUMBER = people_num
