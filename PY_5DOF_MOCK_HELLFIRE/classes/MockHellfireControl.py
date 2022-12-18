import numpy as np
import utility.loggingFxns as lf

STD_GRAV = 9.81 # m/s^2

class MockHellfireControl:

    def __init__(self, ID):

        self.TIME             = 0.0 # s
        self.TIME_STEP        = (1 / 600.0) # s
        self.NEXT_UPDATE_TIME = self.TIME + self.TIME_STEP
        self.PITCH_RATE_LIM   = 10.0 # rad/s
        self.PITCH_PROP_GAIN  = 0.25 # 1/s
        self.YAW_RATE_LIM     = 10.0 # rad/s
        self.YAW_PROP_GAIN    = 0.25 # 1/s
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
        self,
        NORM_COMM,
        SIDE_COMM,
        PITCH_RATE,
        YAW_RATE,
        SPD
    ):

        PITCH_FIN_COMM = None
        YAW_FIN_COMM   = None

        PITCH_RATE_COMM      = -1.0 * NORM_COMM * 8 / SPD
        SIGN_PITCH_RATE_COMM = np.sign(PITCH_RATE_COMM)
        if np.abs(PITCH_RATE_COMM) > self.PITCH_RATE_LIM:
            PITCH_RATE_COMM  = SIGN_PITCH_RATE_COMM * self.PITCH_RATE_LIM
        PITCH_PROP_ERR       = PITCH_RATE_COMM - PITCH_RATE + (STD_GRAV/SPD)
        PITCH_FIN_COMM       = self.PITCH_PROP_GAIN * PITCH_PROP_ERR

        YAW_RATE_COMM      = -1.0 * SIDE_COMM * 8 / SPD
        SIGN_YAW_RATE_COMM = np.sign(YAW_RATE_COMM)
        if np.abs(YAW_RATE_COMM) > self.YAW_RATE_LIM:
            YAW_RATE_COMM  = SIGN_YAW_RATE_COMM * self.YAW_RATE_LIM
        YAW_PROP_ERR       = YAW_RATE_COMM - YAW_RATE
        YAW_FIN_COMM       = self.YAW_PROP_GAIN * YAW_PROP_ERR

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
