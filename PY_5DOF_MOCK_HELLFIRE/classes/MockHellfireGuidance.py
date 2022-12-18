import numpy as np
import utility.loggingFxns as lf

class MockHellfireGuidance:

    def __init__(self, ID, LOA):

        self.TIME             = 0.0 # s
        self.TIME_STEP        = (1 / 100.0) # s
        self.NEXT_UPDATE_TIME = self.TIME + self.TIME_STEP
        self.MID_GUIDE_LIM    = 25.0
        self.TERM_GUIDE_LIM   = 100.0
        self.LINE_OF_ATTACK   = LOA
        self.LOA_GAIN         = 1.5
        self.PRO_NAV_GAIN     = 4.0
        self.NORM_COMM        = 0.0
        self.SIDE_COMM        = 0.0
        self.TGO              = -1.0
        self.FLU_REL_POS      = np.zeros(3)

        self.STATE = {
            "NORM_COMM": self.NORM_COMM,
            "SIDE_COMM": self.SIDE_COMM,
            "TGO": self.TGO,
            "FLU_REL_POS_X": self.FLU_REL_POS[0],
            "FLU_REL_POS_Y": self.FLU_REL_POS[1],
            "FLU_REL_POS_Z": self.FLU_REL_POS[2]
        }

        lf.writeHeader(self.STATE, self.LOGFILE)
        lf.writeData(self.STATE, self.LOGFILE)

        print("MOCK HELLFIRE GUIDANCE LOADED")