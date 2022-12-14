import numpy as np
import numpy.linalg as la
import utility.loggingFxns as lf
import utility.unitVector as uv
from utility.arrayFxns import projection

class MockHellfireGuidance:

    def __init__(self, ID, LOA):

        # COMPONENT CLASS BEHAVIOR.
        self.TIME             = 0.0 # s
        self.TIME_STEP        = (1 / 100.0) # s
        self.NEXT_UPDATE_TIME = self.TIME + self.TIME_STEP # s

        # CLASS MEMBERS.
        self.LOOP_COUNT       = int(0) # counter
        self.GUIDE_FLAG       = int(0) # 0 = LOA, 1 = PRO_NAV
        self.MID_GUIDE_LIM    = 30.0 # m/s^2
        self.TERM_GUIDE_LIM   = 100.0 # m/s^2
        self.LINE_OF_ATTACK   = LOA # nd
        self.LOA_GAIN         = 1.5 # nd
        self.PRO_NAV_GAIN     = 4.0 # nd
        self.NORM_COMM        = 0.0 # m/s^2
        self.SIDE_COMM        = 0.0 # m/s^2
        self.TGO              = -1.0 # s
        self.FLU_REL_POS      = np.zeros(3) # m

        # LOGGING.
        self.STATE = {
            "TIME": self.TIME,
            "NORM_COMM": self.NORM_COMM,
            "SIDE_COMM": self.SIDE_COMM,
            "TGO": self.TGO,
            "FLU_REL_POS_X": self.FLU_REL_POS[0],
            "FLU_REL_POS_Y": self.FLU_REL_POS[1],
            "FLU_REL_POS_Z": self.FLU_REL_POS[2],
            "TGT_POS_X": 0.0,
            "TGT_POS_Y": 0.0,
            "TGT_POS_Z": 0.0
        }

        self.LOGFILE = open(f"PY_5DOF_MOCK_HELLFIRE/data/{ID}.txt", "w")

        lf.writeHeader(self.STATE, self.LOGFILE)
        lf.writeData(self.STATE, self.LOGFILE)

        # REPORT.
        print("MOCK HELLFIRE GUIDANCE LOADED")

    def update(
        self,
        ENU_TO_FLU,
        ENU_POS,
        ENU_VEL,
        TGT_POS,
        TGT_VEL
    ):

        # KINEMATIC TRUTH SEEKER.
        self.FLU_REL_POS = ENU_TO_FLU @ (TGT_POS - ENU_POS) # m
        FLU_REL_POS_U    = uv.unitvector(self.FLU_REL_POS) # nd
        FLU_REL_POS_M    = la.norm(self.FLU_REL_POS) # m
        CLOSING_VEL      = ENU_TO_FLU @ (TGT_VEL - ENU_VEL) # m/s
        CLOSING_SPD      = la.norm(CLOSING_VEL) # m/s
        self.TGO         = FLU_REL_POS_M / CLOSING_SPD # seconds

        # CHECK FOR A TARGET THAT IS RETREATING.
        # ASSUMES GUIDING FROM ZERO VELOCITY (I.E. LAUNCH).
        if self.LOOP_COUNT < 10 and self.GUIDE_FLAG == 0:
            self.LOOP_COUNT       += 1
            if CLOSING_VEL[0] > 0:
                # SWITCH TO PRONAV FOR A RETREATING TARGET.
                self.GUIDE_FLAG = 1

        # PROPORTIONAL GUIDANCE.
        if self.TGO < 5.0 or self.GUIDE_FLAG == 1:
        # if True:
            T1             = np.cross(self.FLU_REL_POS, CLOSING_VEL)
            T2             = np.dot(self.FLU_REL_POS, self.FLU_REL_POS)
            OMEGA          = T1 / T2 # rad/s
            T3             = 1.0 * self.PRO_NAV_GAIN * CLOSING_SPD * FLU_REL_POS_U
            COMM           = np.cross(T3, OMEGA) # m/s^2
            self.NORM_COMM = -COMM[2] # m/s^2
            self.SIDE_COMM = -COMM[1] # m/s^2
            AMAG           = np.sqrt(self.NORM_COMM ** 2 + self.SIDE_COMM ** 2) # m/s^2
            TRIG_RATIO     = np.arctan2(self.NORM_COMM, self.SIDE_COMM) # nd
            if AMAG > self.TERM_GUIDE_LIM:
                AMAG       = self.TERM_GUIDE_LIM # m/s^2
            self.NORM_COMM = AMAG * np.sin(TRIG_RATIO) # m/s^2
            self.SIDE_COMM = AMAG * np.cos(TRIG_RATIO) # m/s^2

        # LINE OF ATTACK GUIDANCE.
        else:
            LOS_VEL        = projection(FLU_REL_POS_U, CLOSING_VEL) # m/s
            LOA_VEL        = projection(self.LINE_OF_ATTACK, CLOSING_VEL) # m/s
            G              = 1 - np.exp(-0.001 * FLU_REL_POS_M) # nd
            self.NORM_COMM = -self.LOA_GAIN * (LOS_VEL[2] + G * LOA_VEL[2]) # m/s^2
            self.SIDE_COMM = -self.LOA_GAIN * (LOS_VEL[1] + G * LOA_VEL[1]) # m/s^2
            AMAG           = np.sqrt(self.NORM_COMM ** 2 + self.SIDE_COMM ** 2) # m/s^2
            TRIG_RATIO     = np.arctan2(self.NORM_COMM, self.SIDE_COMM) # nd
            if AMAG > self.MID_GUIDE_LIM:
                AMAG       = self.MID_GUIDE_LIM # m/s^2
            self.NORM_COMM = AMAG * np.sin(TRIG_RATIO) # m/s^2
            self.SIDE_COMM = AMAG * np.cos(TRIG_RATIO) # m/s^2

        # UPDATE TIME.
        self.TIME             += self.TIME_STEP
        self.NEXT_UPDATE_TIME = self.TIME + self.TIME_STEP

        # GRAB STATE.
        self.STATE = {
            "TIME": self.TIME,
            "NORM_COMM": self.NORM_COMM,
            "SIDE_COMM": self.SIDE_COMM,
            "TGO": self.TGO,
            "FLU_REL_POS_X": self.FLU_REL_POS[0],
            "FLU_REL_POS_Y": self.FLU_REL_POS[1],
            "FLU_REL_POS_Z": self.FLU_REL_POS[2],
            "TGT_POS_X": TGT_POS[0],
            "TGT_POS_Y": TGT_POS[1],
            "TGT_POS_Z": TGT_POS[2]
        }

        # LOG DATA.
        lf.writeData(self.STATE, self.LOGFILE)










