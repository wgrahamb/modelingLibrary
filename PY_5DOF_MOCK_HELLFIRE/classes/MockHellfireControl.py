import numpy as np
import utility.loggingFxns as lf

STD_GRAV = 9.81 # m/s^2

class MockHellfireControl:

    def __init__(self, ID):

        self.TIME             = 0.0 # s
        self.TIME_STEP        = (1 / 600.0) # s
        self.NEXT_UPDATE_TIME = self.TIME + self.TIME_STEP

        self.PITCH_FIN_COMM   = 0.0 # rad

        self.YAW_FIN_COMM     = 0.0 # rad

        self.LOGFILE = open(f"PY_5DOF_MOCK_HELLFIRE/data/{ID}.txt", "w")

        self.STATE = {
            "TIME": self.TIME,
            "PITCH_FIN_COMM_RAD": self.PITCH_FIN_COMM,
            "YAW_FIN_COMM_RAD": self.YAW_FIN_COMM,
        }

        lf.writeHeader(self.STATE, self.LOGFILE)
        lf.writeData(self.STATE, self.LOGFILE)

        print("MOCK HELLFIRE CONTROL LOADED")

    def update(
        self
    ):

        PITCH_FIN_COMM = None
        YAW_FIN_COMM   = None

        self.PITCH_FIN_COMM = PITCH_FIN_COMM
        self.YAW_FIN_COMM   = YAW_FIN_COMM

        self.TIME             += self.TIME_STEP
        self.NEXT_UPDATE_TIME = self.TIME + self.TIME_STEP

        self.STATE = {
            "TIME": self.TIME,
            "PITCH_FIN_COMM_RAD": self.PITCH_FIN_COMM,
            "YAW_FIN_COMM_RAD": self.YAW_FIN_COMM,
        }

        lf.writeData(self.STATE, self.LOGFILE)
