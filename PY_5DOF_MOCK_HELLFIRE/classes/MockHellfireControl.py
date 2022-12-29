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
        self.FIN_LIM          = 12.0 # deg

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

        # From Zarchan. An unexplained gain.
        # Probably from the algebra in the transfer function.
        A_GAIN = 1845

        WCR  = 150.0
        TAU  = 0.1
        ZETA = 0.7

        # Yaw control.
        YB = -1 * Q * REF_AREA * CYB / (MASS * SPD)
        YD = -1 * Q * REF_AREA * CYD / (MASS * SPD)
        NB = Q * REF_AREA * REF_DIAM * CNB / TMOI
        ND = Q * REF_AREA * REF_DIAM * CND / TMOI

        YAW_OMEGAZ  = np.sqrt((NB*YD-YB*ND)/YD)
        YAW_OMEGAAF = np.sqrt(-NB)
        YAW_ZETAAF  = 0.5*YAW_OMEGAAF*YB/NB
        YAW_K1      = -SPD*(NB*YD-ND*YB)/(A_GAIN*NB)
        YAW_TA      = ND/(NB*YD-ND*YB)
        YAW_K3      = A_GAIN*YAW_K1/SPD
        YAW_W       = (TAU*WCR*(1+2.*YAW_ZETAAF*YAW_OMEGAAF/WCR)-1)/(2*ZETA*TAU)
        YAW_W0      = YAW_W/np.sqrt(TAU*WCR)
        YAW_Z0      = .5*YAW_W0*(2*ZETA/YAW_W+TAU-YAW_OMEGAAF**2/(YAW_W0*YAW_W0*WCR))
        YAW_XKC     = (-YAW_W0**2/YAW_OMEGAZ**2-1.+2.*YAW_Z0*YAW_W0*YAW_TA)/(1.-2.*YAW_Z0*YAW_W0*YAW_TA+YAW_W0*YAW_W0*YAW_TA*YAW_TA)
        YAW_XKA     = YAW_K3/(YAW_K1*YAW_XKC)
        YAW_XK0     = -YAW_W*YAW_W/(TAU*YAW_OMEGAAF*YAW_OMEGAAF)
        YAW_XK      = YAW_XK0/(YAW_K1*(1+YAW_XKC))
        YAW_WI      = YAW_XKC*YAW_TA*YAW_W0*YAW_W0/(1+YAW_XKC+YAW_W0**2/YAW_OMEGAZ**2)
        YAW_KR      = YAW_XK/(YAW_XKA*YAW_WI)
        YAW_KDC     = (1 - YAW_KR * YAW_K3) / (YAW_K1 * YAW_KR)

        # YAW_KR  = 0.15
        # YAW_K1  = -1.0 * SPD * ((NB * YD - YB * ND) / (A_GAIN * NB))
        # YAW_K3  = A_GAIN * YAW_K1 / SPD
        # YAW_KDC = (1 - YAW_KR * YAW_K3) / (YAW_K1 * YAW_KR)

        YAW_G_COMM        = (SIDE_COMM / STD_GRAV) * FT_TO_M
        self.YAW_FIN_COMM = YAW_KR * (YAW_KDC * YAW_G_COMM + np.degrees(RRATE))
        TEMP              = np.sign(self.YAW_FIN_COMM)
        if np.abs(self.YAW_FIN_COMM) > self.FIN_LIM:
            self.YAW_FIN_COMM = TEMP * self.FIN_LIM

        # Pitch control.
        ZA = -1 * Q * REF_AREA * CZA / (MASS * SPD)
        ZD = -1 * Q * REF_AREA * CZD / (MASS * SPD)
        MA = Q * REF_AREA * REF_DIAM * CMA / TMOI
        MD = Q * REF_AREA * REF_DIAM * CMD / TMOI

        PITCH_OMEGAZ  = np.sqrt((MA*ZD-ZA*MD)/ZD)
        PITCH_OMEGAAF = np.sqrt(-MA)
        PITCH_ZETAAF  = 0.5*PITCH_OMEGAAF*ZA/MA
        PITCH_K1      = -SPD*(MA*ZD-MD*ZA)/(A_GAIN*MA)
        PITCH_TA      = MD/(MA*ZD-MD*ZA)
        PITCH_K3      = A_GAIN*PITCH_K1/SPD
        PITCH_W       = (TAU*WCR*(1+2.*PITCH_ZETAAF*PITCH_OMEGAAF/WCR)-1)/\
            (2*ZETA*TAU)
        PITCH_W0      = PITCH_W/np.sqrt(TAU*WCR)
        PITCH_Z0      = .5*PITCH_W0*(2*ZETA/PITCH_W+TAU-PITCH_OMEGAAF**2/\
            (PITCH_W0*PITCH_W0*WCR))
        PITCH_XKC     = (-PITCH_W0**2/PITCH_OMEGAZ**2-1.+2.*PITCH_Z0*\
            PITCH_W0*PITCH_TA)/(1.-2.*PITCH_Z0*PITCH_W0*PITCH_TA+PITCH_W0\
            *PITCH_W0*PITCH_TA*PITCH_TA)
        PITCH_XKA     = PITCH_K3/(PITCH_K1*PITCH_XKC)
        PITCH_XK0     = -PITCH_W*PITCH_W/(TAU*PITCH_OMEGAAF*PITCH_OMEGAAF)
        PITCH_XK      = PITCH_XK0/(PITCH_K1*(1+PITCH_XKC))
        PITCH_WI      = PITCH_XKC*PITCH_TA*PITCH_W0*PITCH_W0/\
            (1+PITCH_XKC+PITCH_W0**2/PITCH_OMEGAZ**2)
        PITCH_KR      = PITCH_XK/(PITCH_XKA*PITCH_WI)
        PITCH_KDC     = (1 - PITCH_KR * PITCH_K3) / (PITCH_K1 * PITCH_KR)

        # PITCH_KR  = 0.15
        # PITCH_K1  = -1 * SPD * ((MA * ZD - ZA * MD) / (A_GAIN * MA))
        # PITCH_K3  = A_GAIN * PITCH_K1 / SPD
        # PITCH_KDC = (1 - PITCH_KR * PITCH_K3) / (PITCH_K1 * PITCH_KR)

        PITCH_G_COMM        = (NORM_COMM / STD_GRAV) * FT_TO_M
        self.PITCH_FIN_COMM = PITCH_KR * (PITCH_KDC * PITCH_G_COMM + \
            np.degrees(QRATE))
        TEMP                = np.sign(self.PITCH_FIN_COMM)
        if np.abs(self.PITCH_FIN_COMM) > self.FIN_LIM:
            self.PITCH_FIN_COMM = TEMP * self.FIN_LIM

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
