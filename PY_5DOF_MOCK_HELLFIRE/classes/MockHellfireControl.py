import numpy as np
import utility.loggingFxns as lf

FT_TO_M  = 0.3048
STD_GRAV = 9.81

class MockHellfireControl:

    def __init__(self, ID):

        self.TIME             = 0.0 # s
        self.TIME_STEP        = (1 / 100.0) # s
        self.NEXT_UPDATE_TIME = self.TIME + self.TIME_STEP # s
        self.PITCH_FIN_COMM   = 0.0 # deg
        self.YAW_FIN_COMM     = 0.0 # deg
        self.FIN_LIM          = 8.0 # deg

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
        REF_DIAM, # m^2
        REF_AREA, # m^2
        Q, # pascals
        MASS, # kg
        SPD, # m/s
        TMOI, # kg*m^2
        RRATE, # rad/s
        QRATE, # rad/s
        CYB, # 1/deg
        CYD, # 1/deg
        CNB, # 1/deg
        CND, # 1/deg
        CZA, # 1/deg
        CZD, # 1/deg
        CMA, # 1/deg
        CMD, # 1/deg
        NORM_COMM, # m/s^2
        SIDE_COMM # m/s^2
    ):

        # Yaw control.
        YB = -1 * Q * REF_AREA * CYB / (MASS * SPD)
        YD = -1 * Q * REF_AREA * CYD / (MASS * SPD)
        NB = Q * REF_AREA * REF_DIAM * CNB / TMOI
        ND = Q * REF_AREA * REF_DIAM * CND / TMOI

        YAW_KR  = 0.15
        YAW_K1  = -1.0 * SPD * ((NB * YD - YB * ND) / (1845 * NB))
        YAW_K3  = 1845 * YAW_K1 / SPD
        YAW_KDC = (1 - YAW_KR * YAW_K3) / (YAW_K1 * YAW_KR)

        YAW_G_COMM        = (SIDE_COMM / STD_GRAV) * FT_TO_M
        self.YAW_FIN_COMM = YAW_KR * (YAW_KDC * YAW_G_COMM + np.degrees(RRATE))

        # Pitch control.
        ZA = -1 * Q * REF_AREA * CZA / (MASS * SPD)
        ZD = -1 * Q * REF_AREA * CZD / (MASS * SPD)
        MA = Q * REF_AREA * REF_DIAM * CMA / TMOI
        MD = Q * REF_AREA * REF_DIAM * CMD / TMOI

        PITCH_KR  = 0.15
        PITCH_K1  = -1 * SPD * ((MA * ZD - ZA * MD) / (1845 * MA))
        PITCH_K3  = 1845 * PITCH_K1 / SPD
        PITCH_KDC = (1 - PITCH_KR * PITCH_K3) / (PITCH_K1 * PITCH_KR)

        PITCH_G_COMM        = (NORM_COMM / STD_GRAV) * FT_TO_M
        self.PITCH_FIN_COMM = PITCH_KR * (PITCH_KDC * PITCH_G_COMM + \
            np.degrees(QRATE))

        # Update time.
        self.TIME             += self.TIME_STEP
        self.NEXT_UPDATE_TIME = self.TIME + self.TIME_STEP

        # Grab state.
        self.STATE = {
            "TIME": self.TIME,
            "PITCH_FIN_COMM_DEG": self.PITCH_FIN_COMM,
            "YAW_FIN_COMM_DEG": self.YAW_FIN_COMM,
        }

        # Log data.
        lf.writeData(self.STATE, self.LOGFILE)
