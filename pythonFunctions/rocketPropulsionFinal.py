import numpy as np
import copy
import matplotlib.pyplot as plt
from labellines import labelLine # pip install matplotlib-label-lines
import matplotlib
matplotlib.use("WebAgg")

# General constants.
STD_GRAV = 32.2 # ft/s^2

# Functions.
def GET_ISENP_RATIO(G, MACH):
    T1 = 1
    T2 = (G - 1) / 2.0
    T3 = MACH * MACH
    T4 = G / (G - 1)
    PR = (T1 + T2 * T3) ** T4
    return PR

def GET_ATMOSPHERE(ALT, SPD): # feet. ft/s

    P    = None # air pressure in psi
    T    = None # air temperature in Rankine
    RHO  = None # air density in lbm/ft^3
    A    = None # speed of sound in ft/s
    Q    = None # dynamic pressure in psi
    G    = None # gravity in ft/s^2
    MACH = None # mach speed, non dimensional
    RAD  = 2.0856e+7 # radius of earth in feet

    if ALT < 83000:
        P = -4.272981E-14*(ALT**3) + 0.000000008060081*(ALT**2) - 0.0005482655*ALT + 14.692410
    else:
        P = 0.0

    if ALT < 32809:
        T = -1.0 * 0.0036 * ALT + 518.
    else:
        T = 399.0

    if ALT < 82000:
        RHO = (0.00000000001255)*(ALT**2) - (0.0000019453)*ALT + 0.07579
    else:
        RHO = 0.0

    Q     = (0.5 * RHO * SPD * SPD) * (1 / STD_GRAV) * (1.0 / 144.0)
    G     = STD_GRAV * ((RAD / (RAD+ALT)) ** 2)
    A     = np.sqrt((T * 1.4 * 1545.3 * STD_GRAV) / 28.97)
    MACH  = SPD / A

    return P, T, RHO, A, Q, G, MACH

def GET_CD(MACH):
    RET = None
    if MACH < 0.6:
        RET = 0.15
    elif MACH < 1.2:
        RET = -0.12+0.45*MACH
    elif MACH < 1.8:
        RET = 0.76-0.283*MACH
    elif MACH < 4:
        RET = 0.311-0.034*MACH
    else:
        RET = 0.175
    return RET

# ### Simulation input. ###
# #########################################################################################################
# # Baseline. #
# CHECKS_AND_BALANCES     = False # flag for termination checks
# BORE_RADIUS             = 1.0 # radius of the inner bore due to the grain, inches
# END_RADIUS              = 2.375 # radius of the chamber, inches
# INITIAL_GRAIN_LENGTH    = 8.0 # initial length of each individual grain, inches
# THROAT_AREA_0           = 1.0 # initial area of the throat, in^2
# THROAT_DIAM_0           = np.sqrt(THROAT_AREA_0 * 4.0 / np.pi) # initial diameter of the throat, inches
# EPSILON_0               = 4.0 # initial nozzle expansion ratio
# NUMBER_OF_GRAINS        = int(4) # number of grains, count
# GRAIN_TEMP              = 70.0 # grain temperature, Farenheit
# BALLAST_MASS            = 1.0 # rocket mass that seperates at rocket burnout
# PLOT_FLAG               = False # flag to show simulation plots
# PLOT2_FLAG              = False # flag to show temperature plots
# RPT                     = False # flag to show console report at each iteration
# ID                      = "BASE" # identification for plots
# #########################################################################################################

# ### Simulation input. ###
# #########################################################################################################
# # 5000 Feet. #
# CHECKS_AND_BALANCES     = True # flag for termination checks
# BORE_RADIUS             = 1.0 # radius of the inner bore due to the grain, inches
# END_RADIUS              = 2.1 # radius of the chamber, inches
# INITIAL_GRAIN_LENGTH    = 3.2704 # initial length of each individual grain, inches
# PLOT_FLAG               = True # flag to show simulation plots
# PLOT2_FLAG              = False # flag to show temperature plots
# RPT                     = False # flag to show console report at each iteration
# THROAT_AREA_0           = 0.6512 # initial area of the throat, in^2
# THROAT_DIAM_0           = np.sqrt(THROAT_AREA_0 * 4.0 / np.pi) # initial diameter of the throat, inches
# EPSILON_0               = 2.0 # initial nozzle expansion ratio
# NUMBER_OF_GRAINS        = int(2) # number of grains, count
# GRAIN_TEMP              = 70.0 # grain temperature, Farenheit
# BALLAST_MASS            = 1.0 # rocket mass that seperates at rocket burnout
# ID                      = "5K FT" # identification for plots
# #########################################################################################################

# ### Simulation input. ###
# #########################################################################################################
# # 10000 Feet, 30F. #
# CHECKS_AND_BALANCES     = True # flag for termination checks
# BORE_RADIUS             = 1.0 # radius of the inner bore due to the grain, inches
# END_RADIUS              = 2.1 # radius of the chamber, inches
# INITIAL_GRAIN_LENGTH    = 3.2704 # initial length of each individual grain, inches
# PLOT_FLAG               = False # flag to show simulation plots
# PLOT2_FLAG              = False # flag to show temperature plots
# RPT                     = False # flag to show console report at each iteration
# THROAT_AREA_0           = 1.0 # initial area of the throat, in^2
# THROAT_DIAM_0           = np.sqrt(THROAT_AREA_0 * 4.0 / np.pi) # initial diameter of the throat, inches
# EPSILON_0               = 2.0 # initial nozzle expansion ratio
# NUMBER_OF_GRAINS        = int(3) # number of grains, count
# GRAIN_TEMP              = 30.0 # grain temperature, Farenheit
# BALLAST_MASS            = 1.0 # rocket mass that seperates at rocket burnout
# ID                      = "10K FT, 30F" # identification for plots
# #########################################################################################################

# ### Simulation input. ###
# #########################################################################################################
# # 10000 Feet, 70F. #
# CHECKS_AND_BALANCES     = True # flag for termination checks
# BORE_RADIUS             = 1.0 # radius of the inner bore due to the grain, inches
# END_RADIUS              = 2.1 # radius of the chamber, inches
# INITIAL_GRAIN_LENGTH    = 3.2704 # initial length of each individual grain, inches
# PLOT_FLAG               = True # flag to show simulation plots
# PLOT2_FLAG              = False # flag to show temperature plots
# RPT                     = False # flag to show console report at each iteration
# THROAT_AREA_0           = 1.0 # initial area of the throat, in^2
# THROAT_DIAM_0           = np.sqrt(THROAT_AREA_0 * 4.0 / np.pi) # initial diameter of the throat, inches
# EPSILON_0               = 2.0 # initial nozzle expansion ratio
# NUMBER_OF_GRAINS        = int(3) # number of grains, count
# GRAIN_TEMP              = 70.0 # grain temperature, Farenheit
# BALLAST_MASS            = 1.0 # rocket mass that seperates at rocket burnout
# ID                      = "10K FT, 70F" # identification for plots
# #########################################################################################################

# ### Simulation input. ###
# #########################################################################################################
# # 10000 Feet, 120F. #
# CHECKS_AND_BALANCES     = True # flag for termination checks
# BORE_RADIUS             = 1.0 # radius of the inner bore due to the grain, inches
# END_RADIUS              = 2.1 # radius of the chamber, inches
# INITIAL_GRAIN_LENGTH    = 3.2704 # initial length of each individual grain, inches
# PLOT_FLAG               = False # flag to show simulation plots
# PLOT2_FLAG              = False # flag to show temperature plots
# RPT                     = False # flag to show console report at each iteration
# THROAT_AREA_0           = 1.0 # initial area of the throat, in^2
# THROAT_DIAM_0           = np.sqrt(THROAT_AREA_0 * 4.0 / np.pi) # initial diameter of the throat, inches
# EPSILON_0               = 2.0 # initial nozzle expansion ratio
# NUMBER_OF_GRAINS        = int(3) # number of grains, count
# GRAIN_TEMP              = 120.0 # grain temperature, Farenheit
# BALLAST_MASS            = 1.0 # rocket mass that seperates at rocket burnout
# ID                      = "10K FT, 120F" # identification for plots
# #########################################################################################################

# ### Simulation input. ###
# #########################################################################################################
# # 15000 Feet. #
# CHECKS_AND_BALANCES     = True # flag for termination checks
# BORE_RADIUS             = 1.0 # radius of the inner bore due to the grain, inches
# END_RADIUS              = 2.1 # radius of the chamber, inches
# INITIAL_GRAIN_LENGTH    = 3.2704 # initial length of each individual grain, inches
# PLOT_FLAG               = True # flag to show simulation plots
# PLOT2_FLAG              = False # flag to show temperature plots
# RPT                     = False # flag to show console report at each iteration
# THROAT_AREA_0           = 1.0 # initial area of the throat, in^2
# THROAT_DIAM_0           = np.sqrt(THROAT_AREA_0 * 4.0 / np.pi) # initial diameter of the throat, inches
# EPSILON_0               = 2.268 # initial nozzle expansion ratio
# NUMBER_OF_GRAINS        = int(4) # number of grains, count
# GRAIN_TEMP              = 70.0 # grain temperature, Farenheit
# BALLAST_MASS            = 1.0 # rocket mass that seperates at rocket burnout
# ID                      = "15K FT" # identification for plots
# #########################################################################################################

### Simulation input. ###
#########################################################################################################
# Test. #
CHECKS_AND_BALANCES     = True # flag for termination checks
PLOT_FLAG               = False # flag to show simulation plots
PLOT2_FLAG              = False # flag to show temperature plots
RPT                     = False # flag to show console report at each iteration
GRAIN_TEMP              = 70.0 # grain temperature, Farenheit

ID                      = "TEST" # identification for plots
NUMBER_OF_GRAINS        = int(3) # number of grains, count

BORE_RADIUS             = 0.6444 # radius of the inner bore due to the grain, inches
END_RADIUS              = 1.8777 # radius of the chamber, inches
INITIAL_GRAIN_LENGTH    = 3.3000 # initial length of each individual grain, inches
THROAT_AREA_0           = 0.4945 # initial area of the throat, in^2
THROAT_DIAM_0           = np.sqrt(THROAT_AREA_0 * 4.0 / np.pi) # initial diameter of the throat, inches
EPSILON_0               = 3.5500 # initial nozzle expansion ratio
BALLAST_MASS            = 0.2741 # rocket mass that seperates at rocket burnout
#########################################################################################################

# Propellant characteristics.
CSTAR                   = 5210.0 # characteristic velocity, ft/s
GAMMA                   = 1.25 # specific heat ratio, non dimensional
BURNING_RATE_CONSTANT   = 0.03 # burning rate constant, (in/s) * [(psi)] ** (-n)
MOTOR_PARAM             = 0.001 # motor parameter, 1/Farenheit
NOMINAL_TEMP            = 70.0 # nominal temperature, Farenheit
PROPELLANT_DENSITY      = 0.065 # density of the propellant grain, lbm/in^3
NCONST                  = 0.35 # constant for burning rate

# Rocket constants.
GRAIN_BUFFER            = 0.125 # required buffer between each grain, inches
EXIT_AREA               = EPSILON_0 * THROAT_AREA_0 # nozzle exit area, in^2
EXIT_DIAM               = np.sqrt(EXIT_AREA * 4.0 / np.pi) # nozzle exit diameter, inches
ROCKET_DIAM             = 6.19 # reference diameter, inches
ROCKET_AREA             = (np.pi * (ROCKET_DIAM ** 2) / 4.0) / (12.0 * 12.0) # forward area of the rocket, in^2
ROCKET_MASS             = 40.0 # inert and payload mass of the rocket, lbm

# Initialize data variables.
ITERATION         = None
WEB               = None
AREA_BURN         = None
PROP_MASS         = None
TOF               = None
THROAT_DIAM       = None
THROAT_AREA       = None
CHAMBER_PRESSURE  = None
EXIT_MACH         = None
CFV               = None
CF                = None
THRUST            = None
IMPULSE_BIT       = None
BURNING_RATE      = None
TIME_STEP         = None
DELTA_THROAT_DIAM = None
EPSILON           = None
TOTAL_MASS        = None
SPD               = None
HGT               = None
P                 = None
T                 = None
A                 = None
MACH              = None
CD                = None
RHO               = None
THRUST_ACC        = None
DRAG_ACC          = None
ACC               = None
ACC_G             = None
CF_PLOT           = None
CF_MIN            = None

# Check to see if the nozzle exit area is larger than the rocket diameter.
if EXIT_DIAM > ROCKET_DIAM:
    if CHECKS_AND_BALANCES:
        print("Rocket nozzle too large. Terminating.")
        exit(0)

# Check to see that the bore area is at least twice the throat area.
AP_OVER_AT = (np.pi * (BORE_RADIUS ** 2)) / THROAT_AREA_0
if AP_OVER_AT < 2.0:
    if CHECKS_AND_BALANCES:
        print("Initial grain area is not twice the initial throat area. Terminating.")
        exit(0)

# Calculate mass of the motor case. Check to see if it exceeds thresholds.
MOTOR_CASE_LENGTH = NUMBER_OF_GRAINS * (INITIAL_GRAIN_LENGTH + GRAIN_BUFFER)
if MOTOR_CASE_LENGTH > 34.0:
    if CHECKS_AND_BALANCES:
        print("Motor Case Length Threshold Exceeded. Terminating.")
        exit(0)
MOTOR_CASE_MASS = 0.25 * MOTOR_CASE_LENGTH

# Create a list of set web distances in inches.
WEB_STEP = 0.01
C1       = END_RADIUS - BORE_RADIUS
C2       = INITIAL_GRAIN_LENGTH / 2
MAXWEB   = None
if C1 <= C2:
    MAXWEB = C1
else:
    MAXWEB = C2
WEBS = [0.0]
while True:
    NEWWEB = round((WEBS[-1] + WEB_STEP), 2)
    if NEWWEB >= MAXWEB:
        NEWWEB = MAXWEB
        WEBS.append(NEWWEB)
        break
    WEBS.append(NEWWEB)

# Initialize the atmosphere.
ATMOS = GET_ATMOSPHERE(0.0, 0.0)
P     = ATMOS[0]
T     = ATMOS[1]
RHO   = ATMOS[2]
A     = ATMOS[3]
Q     = ATMOS[4]
G     = ATMOS[5]
MACH  = ATMOS[6]

# Initialize state.
ITERATION         = int(0) # loop count
TIME_STEP         = 0.0 # seconds
TOF               = 0.0 # seconds
HGT               = 0.0 # ft
SPD               = 0.0 # ft/s
ACC               = 0.0 # ft/s^2
THROAT_AREA       = copy.deepcopy(THROAT_AREA_0) # in^2
THROAT_DIAM       = copy.deepcopy(THROAT_DIAM_0)
EPSILON           = copy.deepcopy(EPSILON_0) # nd
DELTA_THROAT_DIAM = 0.0 # inches

LAST_SPD          = None # ft/s
LAST_THRUST       = None # lbf
THIS_THRUST       = 0.0 # lbf
LAST_TIME         = None # seconds
THIS_TIME         = 0.0 # seconds

# Checks and flags.
MAX_CHAMBER_PRESSURE = -1.0 # psi
MAX_THRUST           = -1.0 # lbf
BURNOUT_HGT          = -1.0 # ft
BURNOUT_SPD          = -1.0 # ft/s
BURNOUT_ACC          = -1.0 # ft/s^2
BURNOUT_TOF          = -1.0 # seconds
MAX_HGT              = -1.0 # ft
MAX_SPD              = -1.0 # ft/s
MAX_ACC              = -1.0 # ft/s^2

# Data.
DATA = {
    "ITERATION":                  [],
    "WEB_DISTANCE":               [],
    "BURN_AREA":                  [],
    "PROPELLANT_MASS":            [],
    "TIME_OF_FLIGHT":             [],
    "THROAT_DIAMETER":            [],
    "THROAT_AREA":                [],
    "CHAMBER_PRESSURE":           [],
    "EXIT_MACH":                  [],
    "VACUUM_FORCE_COEFFICIENT":   [],
    "FORCE_COEFFICIENT":          [],
    "THRUST":                     [],
    "IMPULSE_BIT":                [],
    "BURNING_RATE":               [],
    "TIME_STEP":                  [],
    "CHANGE_IN_THROAT_DIAMETER":  [],
    "NOZZLE_AREA_RATIO":          [],
    "TOTAL_MASS":                 [],
    "SPEED":                      [],
    "HEIGHT":                     [],
    "AIR_PRESSURE":               [],
    "AIR_TEMPERATURE":            [],
    "SPEED_OF_SOUND":             [],
    "MACH":                       [],
    "DRAG_COEFFICIENT":           [],
    "AIR_DENSITY":                [],
    "ACCELERATION_DUE_TO_THRUST": [],
    "ACCELERATION_DUE_TO_DRAG":   [],
    "TOTAL_ACCELERATION":         [],
    "TOTAL_ACCELERATION_GS":      [],
    "PLOT_FORCE_COEFFICIENT":     [],
    "MININUM_FORCE_COEFFICIENT":  []
}

print("\n")
for INDEX, WEB in enumerate(WEBS):

    ITERATION = INDEX

    # Integrate state.
    LAST_SPD = copy.deepcopy(SPD)
    SPD      += ACC * TIME_STEP
    HGT      += 0.5 * (LAST_SPD + SPD) * TIME_STEP

    # Update the atmosphere model.
    ATMOS = GET_ATMOSPHERE(HGT, SPD)
    P     = ATMOS[0]
    T     = ATMOS[1]
    RHO   = ATMOS[2]
    A     = ATMOS[3]
    Q     = ATMOS[4]
    G     = ATMOS[5]
    MACH  = ATMOS[6]

    # Burn area.
    AREA_END  = 2 * np.pi * (END_RADIUS ** 2 - (BORE_RADIUS + WEB) ** 2)
    AREA_BORE = 2 * np.pi * (BORE_RADIUS + WEB) * (INITIAL_GRAIN_LENGTH - 2.0 * WEB)
    AREA_BURN = NUMBER_OF_GRAINS * (AREA_BORE + AREA_END)

    # Propellant mass.
    T1        = (INITIAL_GRAIN_LENGTH - 2.0 * WEB) * (END_RADIUS ** 2 - (BORE_RADIUS + WEB) ** 2)
    PROP_MASS = T1 * np.pi * PROPELLANT_DENSITY * NUMBER_OF_GRAINS

    # Time of flight.
    TOF += TIME_STEP

    # Throat area.
    THROAT_DIAM += DELTA_THROAT_DIAM
    THROAT_AREA = (np.pi * THROAT_DIAM ** 2) / 4.0

    # Chamber pressure.
    CHAMBER_PRESSURE = (
        (
            BURNING_RATE_CONSTANT \
            * np.exp(MOTOR_PARAM * (GRAIN_TEMP - NOMINAL_TEMP)) \
            * PROPELLANT_DENSITY \
            * CSTAR \
            * (AREA_BURN / THROAT_AREA)
        ) / STD_GRAV
    ) ** (1 / (1-NCONST))

    # Nozzle area ratio.
    EPSILON = EXIT_AREA / THROAT_AREA

    # Exit mach.
    STOP      = 0.000001
    EA        = STOP * 1.1
    EXIT_MACH = 1.5
    I         = 0
    while EA > STOP and I < 100:
        I         += 1
        AFUN      = (2.0 + (GAMMA - 1) * EXIT_MACH * EXIT_MACH) / (GAMMA + 1.0)
        BFUN      = (GAMMA + 1.0) / (2.0 * (GAMMA - 1.0))
        CFUN      = 1.0 / AFUN
        DFUN      = 1.0 / (EXIT_MACH * EXIT_MACH)
        DERFUN    = (AFUN ** BFUN) * (CFUN - DFUN)
        FUNFUN    = (1.0 / EXIT_MACH) * (AFUN ** BFUN) - EPSILON
        AMOLD     = EXIT_MACH
        EXIT_MACH -= (FUNFUN / DERFUN)
        EA        = np.abs((EXIT_MACH - AMOLD) / EXIT_MACH) * 100.0

    # Vacuum force coefficient.
    T1  = 2 * GAMMA * GAMMA / (GAMMA - 1)
    T2  = 2 / (GAMMA + 1)
    T3  = (GAMMA + 1) / (GAMMA - 1)
    T4  = 1
    T5  = 1.0 / (GET_ISENP_RATIO(GAMMA, EXIT_MACH))
    T6  = (GAMMA - 1) / GAMMA
    T7  = (1.0 / (GET_ISENP_RATIO(GAMMA, EXIT_MACH))) * EPSILON
    CFV = np.sqrt(T1 * (T2 ** T3) * (T4 - T5 ** T6)) + T7

    # Force coefficient.
    CF = CFV - (P / CHAMBER_PRESSURE) * EPSILON

    # Thrust.
    THRUST = CF * CHAMBER_PRESSURE * THROAT_AREA

    # Impulse bit.
    LAST_THRUST = THIS_THRUST
    THIS_THRUST = THRUST
    LAST_TIME   = THIS_TIME
    THIS_TIME   = TOF
    IMPULSE_BIT = 0.5 * (LAST_THRUST + THIS_THRUST) * (THIS_TIME - LAST_TIME)

    # Burning rate.
    BURNING_RATE = (
        BURNING_RATE_CONSTANT \
        * np.exp(MOTOR_PARAM * (GRAIN_TEMP - NOMINAL_TEMP)) \
        * (CHAMBER_PRESSURE ** NCONST)
    )

    try:
        TIME_STEP         = (WEBS[INDEX+1]-WEB) / BURNING_RATE
        DELTA_THROAT_DIAM = 0.000087 * TIME_STEP * CHAMBER_PRESSURE
    except:
        DELTA_THROAT_DIAM = 0.0

    # Total mass.
    TOTAL_MASS = PROP_MASS + (MOTOR_CASE_MASS + BALLAST_MASS + ROCKET_MASS)

    # Drag coefficient.
    CD = float(GET_CD(MACH))

    # Acceleration due to thrust.
    THRUST_ACC = (THRUST / TOTAL_MASS) * STD_GRAV

    # Acceleration due to drag.
    DRAG_ACC = ((CD * (ROCKET_AREA * 144.0) * Q) / TOTAL_MASS) * STD_GRAV

    # Total acceleration.
    ACC = THRUST_ACC - DRAG_ACC - STD_GRAV

    # Total acceleration in Gs.
    ACC_G = ACC / STD_GRAV

    # Force coefficient for plot.
    CF_PLOT = CF

    # Force coefficient minimum.
    T1     = -1.0 * 0.0445 * (np.log(EPSILON) ** 2)
    T2     = 0.5324 * np.log(EPSILON)
    T3     = 0.1843
    CF_MIN = T1 + T2 + T3

    # Checks and flags.
    if CHAMBER_PRESSURE > MAX_CHAMBER_PRESSURE:
        MAX_CHAMBER_PRESSURE = CHAMBER_PRESSURE
    if THRUST > MAX_THRUST:
        MAX_THRUST = THRUST
    if HGT > MAX_HGT:
        MAX_HGT = HGT
    if SPD > MAX_SPD:
        MAX_SPD = SPD
    if ACC > MAX_ACC:
        MAX_ACC = ACC
    if CHAMBER_PRESSURE > 1000.0:
        if CHECKS_AND_BALANCES:
            print("Chamber pressure maximum exceeded. Terminating.")
            exit(0)
    if CF < CF_MIN:
        if CHECKS_AND_BALANCES:
            print("Flow seperation. Terminating.")
            exit(0)
    if EPSILON < 1.0:
        if CHECKS_AND_BALANCES:
            print("Area of the nozzle throat exceeds the area of the nozzle exit. Terminating.")
            exit(0)
    if ACC_G > 15.0:
        if CHECKS_AND_BALANCES:
            print("Acceleration of the rocket exceeds 15 Gs. Terminating.")
            exit(0)

    # Report.
    if RPT:
        print(f"ITERATION:                  {ITERATION}")
        print(f"WEB DISTANCE:               {WEB:.6f} IN")
        print(f"BURN AREA:                  {AREA_BURN:.6f} IN^2")
        print(f"PROPELLANT MASS:            {PROP_MASS:.6f} LBM")
        print(f"TIME OF FLIGHT:             {TOF:.6f} SECONDS")
        print(f"THROAT DIAMETER:            {THROAT_DIAM:.6f} IN")
        print(f"THROAT AREA:                {THROAT_AREA:.6f} IN^2")
        print(f"CHAMBER PRESSURE:           {CHAMBER_PRESSURE:.6f} PSI")
        print(f"EXIT MACH:                  {EXIT_MACH:.6f}")
        print(f"VACUUM FORCE COEFFICIENT:   {CFV:.6f}")
        print(f"FORCE COEFFICIENT:          {CF:.6f}")
        print(f"THRUST:                     {THRUST:.6f} LBF")
        print(f"IMPULSE BIT:                {IMPULSE_BIT:.6f} LBF*S")
        print(f"BURNING RATE:               {BURNING_RATE:.6f} IN/S")
        print(f"TIME STEP:                  {TIME_STEP:.6f} SECONDS")
        print(f"CHANGE IN THROAT DIAMETER:  {DELTA_THROAT_DIAM:.6f} IN")
        print(f"NOZZLE AREA RATIO:          {EPSILON:.6f}")
        print(f"TOTAL MASS:                 {TOTAL_MASS:.6f} LBM")
        print(f"SPEED:                      {SPD:.6f} FT/S")
        print(f"HEIGHT:                     {HGT:.6f} FT")
        print(f"AIR PRESSURE:               {P:.6f} PSI")
        print(f"AIR TEMPERATURE:            {T:.6f} RANKINE")
        print(f"SPEED OF SOUND:             {A:.6f} FT/S")
        print(f"MACH:                       {MACH:.6f}")
        print(f"DRAG COEFFICIENT:           {CD:.6f}")
        print(f"AIR DENSITY:                {RHO:.6f} LBM/FT^3")
        print(f"ACCELERATION DUE TO THRUST: {THRUST_ACC:.6f} FT/S^2")
        print(f"ACCELERATION DUE TO DRAG:   {DRAG_ACC:.6f} FT/S^2")
        print(f"TOTAL ACCELERATION:         {ACC:.6f} FT/S^2")
        print(f"TOTAL ACCELERATION:         {ACC_G:.6f} Gs")
        print(f"PLOT FORCE COEFFICIENT      {CF_PLOT:.6F}")
        print(f"MINIMUM FORCE COEFFICIENT:  {CF_MIN:.6f}")
        print("\n")

    # Store data.
    DATA["ITERATION"].append(INDEX)
    DATA["WEB_DISTANCE"].append(WEB)
    DATA["BURN_AREA"].append(AREA_BURN)
    DATA["PROPELLANT_MASS"].append(PROP_MASS)
    DATA["TIME_OF_FLIGHT"].append(TOF)
    DATA["THROAT_DIAMETER"].append(THROAT_DIAM)
    DATA["THROAT_AREA"].append(THROAT_AREA)
    DATA["CHAMBER_PRESSURE"].append(CHAMBER_PRESSURE)
    DATA["EXIT_MACH"].append(EXIT_MACH)
    DATA["VACUUM_FORCE_COEFFICIENT"].append(CFV)
    DATA["FORCE_COEFFICIENT"].append(CF)
    DATA["THRUST"].append(THRUST)
    DATA["IMPULSE_BIT"].append(IMPULSE_BIT)
    DATA["BURNING_RATE"].append(BURNING_RATE)
    DATA["TIME_STEP"].append(TIME_STEP)
    DATA["CHANGE_IN_THROAT_DIAMETER"].append(DELTA_THROAT_DIAM)
    DATA["NOZZLE_AREA_RATIO"].append(EPSILON)
    DATA["TOTAL_MASS"].append(TOTAL_MASS)
    DATA["SPEED"].append(SPD)
    DATA["HEIGHT"].append(HGT)
    DATA["AIR_PRESSURE"].append(P)
    DATA["AIR_TEMPERATURE"].append(T)
    DATA["SPEED_OF_SOUND"].append(A)
    DATA["MACH"].append(MACH)
    DATA["DRAG_COEFFICIENT"].append(CD)
    DATA["AIR_DENSITY"].append(RHO)
    DATA["ACCELERATION_DUE_TO_THRUST"].append(THRUST_ACC)
    DATA["ACCELERATION_DUE_TO_DRAG"].append(DRAG_ACC)
    DATA["TOTAL_ACCELERATION"].append(ACC)
    DATA["TOTAL_ACCELERATION_GS"].append(ACC_G)
    DATA["PLOT_FORCE_COEFFICIENT"].append(CF_PLOT)
    DATA["MININUM_FORCE_COEFFICIENT"].append(CF_MIN)

BURNOUT_HGT = copy.deepcopy(HGT)
BURNOUT_SPD = copy.deepcopy(SPD)
BURNOUT_ACC = copy.deepcopy(ACC)
BURNOUT_TOF = copy.deepcopy(TOF)
ISP         = np.sum(DATA["IMPULSE_BIT"]) / DATA['PROPELLANT_MASS'][0]

# Adjust variables for burnout.
CHAMBER_PRESSURE = 0.0
EXIT_MACH        = 0.0
CFV              = 0.0
CF               = 0.0
THRUST           = 0.0
IMPULSE_BIT      = 0.0
BURNING_RATE     = 0.0
TIME_STEP        = 0.1
THRUST_ACC       = 0.0

while True:

    ITERATION += 1

    # Integrate state.
    LAST_SPD = copy.deepcopy(SPD)
    SPD      += ACC * TIME_STEP
    HGT      += 0.5 * (LAST_SPD + SPD) * TIME_STEP

    # Update the atmosphere model.
    ATMOS = GET_ATMOSPHERE(HGT, SPD)
    P     = ATMOS[0]
    T     = ATMOS[1]
    RHO   = ATMOS[2]
    A     = ATMOS[3]
    Q     = ATMOS[4]
    G     = ATMOS[5]
    MACH  = ATMOS[6]

    # Time of flight.
    TOF += TIME_STEP

    # Drag coefficient.
    CD = float(GET_CD(MACH))

    # Acceleration due to drag.
    DRAG_ACC = ((CD * (ROCKET_AREA * 144.0) * Q) / TOTAL_MASS) * STD_GRAV

    # Total acceleration.
    ACC = THRUST_ACC - DRAG_ACC - STD_GRAV

    # Total acceleration in Gs.
    ACC_G = ACC / STD_GRAV

    # Checks and flags.
    if HGT > MAX_HGT:
        MAX_HGT = HGT
    if SPD > MAX_SPD:
        MAX_SPD = SPD
    if ACC > MAX_ACC:
        MAX_ACC = ACC

    # Report.
    if RPT:
        print(f"ITERATION:                  {ITERATION}")
        print(f"WEB DISTANCE:               {WEB:.6f} IN")
        print(f"BURN AREA:                  {AREA_BURN:.6f} IN^2")
        print(f"PROPELLANT MASS:            {PROP_MASS:.6f} LBM")
        print(f"TIME OF FLIGHT:             {TOF:.6f} SECONDS")
        print(f"THROAT DIAMETER:            {THROAT_DIAM:.6f} IN")
        print(f"THROAT AREA:                {THROAT_AREA:.6f} IN^2")
        print(f"CHAMBER PRESSURE:           {CHAMBER_PRESSURE:.6f} PSI")
        print(f"EXIT MACH:                  {EXIT_MACH:.6f}")
        print(f"VACUUM FORCE COEFFICIENT:   {CFV:.6f}")
        print(f"FORCE COEFFICIENT:          {CF:.6f}")
        print(f"THRUST:                     {THRUST:.6f} LBF")
        print(f"IMPULSE BIT:                {IMPULSE_BIT:.6f} LBF*S")
        print(f"BURNING RATE:               {BURNING_RATE:.6f} IN/S")
        print(f"TIME STEP:                  {TIME_STEP:.6f} SECONDS")
        print(f"CHANGE IN THROAT DIAMETER:  {DELTA_THROAT_DIAM:.6f} IN")
        print(f"NOZZLE AREA RATIO:          {EPSILON:.6f}")
        print(f"TOTAL MASS:                 {TOTAL_MASS:.6f} LBM")
        print(f"SPEED:                      {SPD:.6f} FT/S")
        print(f"HEIGHT:                     {HGT:.6f} FT")
        print(f"AIR PRESSURE:               {P:.6f} PSI")
        print(f"AIR TEMPERATURE:            {T:.6f} RANKINE")
        print(f"SPEED OF SOUND:             {A:.6f} FT/S")
        print(f"MACH:                       {MACH:.6f}")
        print(f"DRAG COEFFICIENT:           {CD:.6f}")
        print(f"AIR DENSITY:                {RHO:.6f} LBM/FT^3")
        print(f"ACCELERATION DUE TO THRUST: {THRUST_ACC:.6f} FT/S^2")
        print(f"ACCELERATION DUE TO DRAG:   {DRAG_ACC:.6f} FT/S^2")
        print(f"TOTAL ACCELERATION:         {ACC:.6f} FT/S^2")
        print(f"TOTAL ACCELERATION:         {ACC_G:.6f} Gs")
        print(f"PLOT FORCE COEFFICIENT      {CF_PLOT:.6F}")
        print(f"MINIMUM FORCE COEFFICIENT:  {CF_MIN:.6f}")
        print("\n")

    # Store data.
    DATA["ITERATION"].append(INDEX)
    DATA["WEB_DISTANCE"].append(WEB)
    DATA["BURN_AREA"].append(AREA_BURN)
    DATA["PROPELLANT_MASS"].append(PROP_MASS)
    DATA["TIME_OF_FLIGHT"].append(TOF)
    DATA["THROAT_DIAMETER"].append(THROAT_DIAM)
    DATA["THROAT_AREA"].append(THROAT_AREA)
    DATA["CHAMBER_PRESSURE"].append(CHAMBER_PRESSURE)
    DATA["EXIT_MACH"].append(EXIT_MACH)
    DATA["VACUUM_FORCE_COEFFICIENT"].append(CFV)
    DATA["FORCE_COEFFICIENT"].append(CF)
    DATA["THRUST"].append(THRUST)
    DATA["IMPULSE_BIT"].append(IMPULSE_BIT)
    DATA["BURNING_RATE"].append(BURNING_RATE)
    DATA["TIME_STEP"].append(TIME_STEP)
    DATA["CHANGE_IN_THROAT_DIAMETER"].append(DELTA_THROAT_DIAM)
    DATA["NOZZLE_AREA_RATIO"].append(EPSILON)
    DATA["TOTAL_MASS"].append(TOTAL_MASS)
    DATA["SPEED"].append(SPD)
    DATA["HEIGHT"].append(HGT)
    DATA["AIR_PRESSURE"].append(P)
    DATA["AIR_TEMPERATURE"].append(T)
    DATA["SPEED_OF_SOUND"].append(A)
    DATA["MACH"].append(MACH)
    DATA["DRAG_COEFFICIENT"].append(CD)
    DATA["AIR_DENSITY"].append(RHO)
    DATA["ACCELERATION_DUE_TO_THRUST"].append(THRUST_ACC)
    DATA["ACCELERATION_DUE_TO_DRAG"].append(DRAG_ACC)
    DATA["TOTAL_ACCELERATION"].append(ACC)
    DATA["TOTAL_ACCELERATION_GS"].append(ACC_G)
    DATA["PLOT_FORCE_COEFFICIENT"].append(CF_PLOT)
    DATA["MININUM_FORCE_COEFFICIENT"].append(CF_MIN)

    if SPD < 0.0:
        break

# Final report.
print(f"FINAL REPORT:")
print(f"REFERENCE AREA:                             {ROCKET_AREA:.6f} FT^2")
print(f"INITIAL PROPELLANT MASS:                    {DATA['PROPELLANT_MASS'][0]:.6f} LBM")
print(f"FINAL THROAT AREA:                          {THROAT_AREA:.6f} IN^2")
print(f"BURN TIME:                                  {BURNOUT_TOF:.6f} SECONDS")
print(f"SPECIFIC IMPULSE:                           {ISP:.6f} SECONDS")
print(f"MAX CHAMBER PRESSURE:                       {MAX_CHAMBER_PRESSURE:.6f} PSI")
print(f"MAX THRUST:                                 {MAX_THRUST:.6f} LBF")
print(f"INITIAL BORE AREA OVER INITIAL THROAT AREA: {AP_OVER_AT:.6f}")
print(f"MAX WEB:                                    {MAXWEB:.6f} IN")
print(f"MASS OF THE MOTOR CASE:                     {MOTOR_CASE_MASS:.6f} LBM")
print(f"INITIAL TOTAL MASS:                         {DATA['TOTAL_MASS'][0]:.6f} LBM")
print(f"BURNOUT HEIGHT:                             {BURNOUT_HGT:.6f} FT")
print(f"BURNOUT SPEED:                              {BURNOUT_SPD:.6f} FT/S")
print(f"BURNOUT ACCELERATION:                       {BURNOUT_ACC:.6f} FT/S^2")
print(f"MAX HEIGHT:                                 {MAX_HGT:.6f} FT")
print(f"MAX SPEED:                                  {MAX_SPD:.6f} FT/S")
print(f"MAX ACCELERATION:                           {MAX_ACC:.6f} FT/S^2")
print(f"MAX ACCELERATION:                           {MAX_ACC/STD_GRAV:.6f} Gs")
print("\n")

if PLOT_FLAG:

    ### Figure One. ###
    ### Cross section of baseline vehicle grain shapes with dimensions. ###
    plt.figure(1, figsize=[7,7])
    plt.xlim([-2, 2])
    plt.ylim([-2, 2])
    plt.xticks([])
    plt.yticks([])

    plt.plot([-1.5, -0.5], [0.0, 0.0], color="b")

    plt.plot([-1.5, -1.5], [0.5, 1.5], color="b")
    plt.plot([-0.5, -0.5], [0.5, 1.5], color="b")
    plt.plot([-1.5, -0.5], [1.5, 1.5], color="b")
    plt.plot([-1.5, -0.5], [0.5, 0.5], color="b")

    plt.plot([-1.5, -1.5], [-0.5, 0.5], color="b")
    plt.plot([-0.5, -0.5], [-0.5, 0.5], color="b")
    plt.plot([-1.5, -0.5], [0.5, 0.5], color="b")
    plt.plot([-1.5, -0.5], [-0.5, -0.5], color="b")

    plt.plot([-1.5, -1.5], [-0.5, -1.5], color="b")
    plt.plot([-0.5, -0.5], [-0.5, -1.5], color="b")
    plt.plot([-1.5, -0.5], [-1.5, -1.5], color="b")
    plt.plot([-1.5, -0.5], [-0.5, -0.5], color="b")

    theta = np.linspace( 0 , 2 * np.pi , 150 )
    radius1 = 0.7
    x1 = radius1 * np.cos( theta )
    y1 = radius1 * np.sin( theta )
    radius2 = 0.35
    x2 = radius2 * np.cos( theta )
    y2 = radius2 * np.sin( theta )
    plt.plot(1+x1, y1, color="b")
    plt.plot(1+x2, y2, color="b")
    plt.annotate(
        "",
        xy=(-1.5, 1.6),
        xytext=(-0.5, 1.6),
        arrowprops={'arrowstyle': '<->'}
    )
    plt.annotate(
        f"{INITIAL_GRAIN_LENGTH:.3f} in",
        xy=(-1.5, 1.7),
        fontsize=11
    )
    plt.annotate(
        "",
        xy=(-0.5, 1.6),
        xytext=(-0.3, 1.6),
        arrowprops={'arrowstyle': '<->'}
    )
    plt.annotate(
        f"{GRAIN_BUFFER:.3f} in",
        xy=(-0.5, 1.7),
        fontsize=11
    )
    plt.annotate(
        "",
        xy=(-0.4, 0),
        xytext=(-0.4, 1.5),
        arrowprops={'arrowstyle': '<->'}
    )
    plt.annotate(
        f"{END_RADIUS:.3f} in",
        xy=(-0.3, 0.75),
        fontsize=11
    )
    plt.annotate(
        "",
        xy=(-0.3, 0),
        xytext=(-0.3, 0.5),
        arrowprops={'arrowstyle': '<->'}
    )
    plt.annotate(
        f"{BORE_RADIUS:.3f} in",
        xy=(-0.2, 0.25),
        fontsize=11
    )
    plt.annotate(
        f"{NUMBER_OF_GRAINS} Grains",
        xy=(-1.2, -1.75),
        fontsize=11
    )
    plt.annotate(
        "",
        xy=(0.2, 1.5),
        xytext=(1, 1.5),
        arrowprops={'arrowstyle': '<->'}
    )
    plt.annotate(
        f"{END_RADIUS:.3f} in",
        xy=(1.1, 1.5),
        fontsize=11
    )
    plt.annotate(
        "",
        xy=(0.6, 1),
        xytext=(1, 1),
        arrowprops={'arrowstyle': '<->'}
    )
    plt.annotate(
        f"{BORE_RADIUS:.3f} in",
        xy=(1.1, 1),
        fontsize=11
    )
    ###################

    ### Figure Two. ###
    ### Mass of propellant as a function of time. ###
    plt.figure(2, figsize=[7,7])
    plt.plot(DATA["TIME_OF_FLIGHT"], DATA["PROPELLANT_MASS"], color="b")
    plt.grid()
    plt.xlim([0.0, BURNOUT_TOF+1.0])
    plt.tick_params(labelsize=12, axis="both")
    plt.xlabel("TIME (SEC)", fontsize=13)
    plt.ylabel("Mp, MASS PROP., (LBm)", fontsize=13)
    plt.title("MASS OF PROPELLANT")
    ###################

    ### Figure Three. ###
    ### Chamber as a function of time. ###
    plt.figure(3, figsize=[7,7])
    plt.plot(DATA["TIME_OF_FLIGHT"], DATA["CHAMBER_PRESSURE"], color="b")
    plt.grid()
    plt.xlim([0.0, BURNOUT_TOF+1.0])
    plt.tick_params(labelsize=12, axis="both")
    plt.xlabel("TIME (SEC)", fontsize=13)
    plt.ylabel("Pc, PRESSURE, (PSIA)", fontsize=13)
    plt.title("CHAMBER PRESSURE")
    ###################

    ### Figure Four. ###
    ### Baseline thrust as a function of time. ###
    plt.figure(4, figsize=[7,7])
    plt.plot(DATA["TIME_OF_FLIGHT"], DATA["THRUST"], color="b")
    plt.grid()
    plt.xlim([0.0, BURNOUT_TOF+1.0])
    plt.tick_params(labelsize=12, axis="both")
    plt.xlabel("TIME (SEC)", fontsize=13)
    plt.ylabel("F, THRUST, (LBF)", fontsize=13)
    plt.title("THRUST")
    ###################

    ### Figure Five. ###
    ### Baseline acceleration as a function of time. ###
    plt.figure(5, figsize=[7,7])
    if ID == "BASE":
        plt.plot(DATA["TIME_OF_FLIGHT"], DATA["TOTAL_ACCELERATION"], color="b")
    else:
        plt.plot(DATA["TIME_OF_FLIGHT"], DATA["TOTAL_ACCELERATION_GS"], color="b")
    plt.grid()
    plt.xlim([0.0, DATA["TIME_OF_FLIGHT"][-1]+1.0])
    plt.tick_params(labelsize=12, axis="both")
    plt.xlabel("TIME (SEC)", fontsize=13)
    if ID == "BASE":
        plt.ylabel("ACCEL, (FT/S^2)", fontsize=13)
    else:
        plt.ylabel("ACCEL, (Gs)", fontsize=13)
    plt.title("TRAJECTORY")
    ###################

    ### Figure Six. ###
    ### Baseline speed as a function of time. ###
    plt.figure(6, figsize=[7,7])
    plt.plot(DATA["TIME_OF_FLIGHT"], DATA["SPEED"], color="b")
    plt.grid()
    plt.xlim([0.0, DATA["TIME_OF_FLIGHT"][-1]+1.0])
    plt.tick_params(labelsize=12, axis="both")
    plt.xlabel("TIME (SEC)", fontsize=13)
    plt.ylabel("VEL, (FT/S)", fontsize=13)
    plt.title("TRAJECTORY")
    ###################

    ### Figure Seven. ###
    ### Baseline altitude as a function of time. ###
    plt.figure(7, figsize=[7,7])
    plt.plot(DATA["TIME_OF_FLIGHT"], DATA["HEIGHT"], color="b")
    plt.grid()
    plt.xlim([0.0, DATA["TIME_OF_FLIGHT"][-1]+1.0])
    plt.tick_params(labelsize=12, axis="both")
    plt.xlabel("TIME (SEC)", fontsize=13)
    plt.ylabel("ALTITUDE, (FT)", fontsize=13)
    plt.title("TRAJECTORY")
    ###################

    ### Figure Eight. ###
    ### Thrust coefficient as a function of nozzle area ratio. ###
    def newtonGetMach_(initialMachGuess, gamma, sigma):
        StopCriteria = 0.000001
        EA = StopCriteria * 1.1
        AM2 = copy.deepcopy(initialMachGuess)
        index = 0
        while EA > StopCriteria and index < 100:
            index += 1
            AFUN = (2.0 + (gamma - 1) * AM2 * AM2) / (gamma + 1.0)
            BFUN = (gamma + 1.0) / (2.0 * (gamma - 1.0))
            CFUN = 1.0 / AFUN
            DFUN = 1.0 / (AM2 * AM2)
            DERFUN = (AFUN ** BFUN) * (CFUN - DFUN)
            FUNFUN = (1.0 / AM2) * (AFUN ** BFUN) - sigma
            AMOLD = AM2
            AM2 -= (FUNFUN / DERFUN)
            EA = np.abs((AM2 - AMOLD) / AM2) * 100.0
        return AM2

    def getNozzAreaRatio_(mach, gamma):
        t1 = (1 / mach)
        t2 = 2 + (gamma - 1) * mach * mach
        t3 = gamma + 1
        t4 = (gamma + 1) / (2.0 * (gamma - 1))
        t = t1 * ((t2 / t3) ** t4)
        return t

    def bisectionGetMach_(low, high, gamma, sigma):
        l = low
        h = high
        index = 0
        while True:
            index += 1
            machGuess = (l + h) / 2.0
            x = getNozzAreaRatio_(machGuess, gamma)
            check = x / sigma
            # print(f"REPORT\n  INDEX {index}\n  LOW {l}\n  HIGH {h}\n  MACH {machGuess:.2f}\n  CALC {x:.2f}\n  SIGMA {sigma:.2f}\n  CHECK {check:.2f}\n")
            if 0.99 < check < 1.01:
                # print(f"Solution found: {machGuess}\n")
                break
            elif check < 0.98:
                l = machGuess
            elif check > 1.02:
                h = machGuess
            if index == 20:
                # print(f"Solution does not converge.\n")
                break
        return machGuess

    def getCfv_(gamma, sigma, PcOverPe):
        t1 = 2 * gamma * gamma / (gamma - 1)
        t2 = 2 / (gamma + 1)
        t3 = (gamma + 1) / (gamma - 1)
        t4 = 1
        t5 = 1.0 / PcOverPe
        t6 = (gamma - 1) / gamma
        t7 = (1.0 / PcOverPe) * sigma
        ret = np.sqrt(t1 * (t2 ** t3) * (t4 - t5 ** t6)) + t7
        return ret

    def getCf_(cfv, PcOverPa, sigma):
        ret = (cfv - (1.0 / PcOverPa) * sigma)
        return ret

    def getIsenPRatio_(gamma, mach):
        t1 = 1
        t2 = (gamma - 1) / 2.0
        t3 = mach * mach
        t4 = gamma / (gamma - 1)
        ret = (t1 + t2 * t3) ** t4
        return ret

    def getCfMin_(sigma):
        t1 = -1.0 * 0.0445 * (np.log(sigma) ** 2)
        t2 = 0.5324 * np.log(sigma)
        t3 = 0.1843
        ret = t1 + t2 + t3
        return ret

    # initialize
    gamma_          = 1.2
    ambPress_       = 14.7
    sigmas_         = np.linspace(1.01, 100, 10000)
    PcOverPaList_   = [2, 4, 10, 25, 75, 150, 500, 1000, 1000000000000]
    lineOfMaxCfsX_  = []
    lineOfMaxCfsY_  = []
    data_           = []
    cfBottomLim_    = 0.6
    cfTopLim_       = 2.0

    # loop over set PcOverPa list
    for index_, PcOverPa_ in enumerate(PcOverPaList_):
        sigmaData_ = []
        cfData_    = []
        flag_      = 0
        for i_, sigma_ in enumerate(sigmas_):
            Pc_       = PcOverPa_ * ambPress_
            Me_       = None
            if PcOverPa_ == 2:
                Me_ = newtonGetMach_(2.0, gamma_, sigma_)
            else:
                Me_ = bisectionGetMach_(0, 5, gamma_, sigma_)
            PcOverPe_ = getIsenPRatio_(gamma_, Me_)
            Pe_       = (1.0 / PcOverPe_) * Pc_
            check_    = np.abs(Pe_ - ambPress_)
            cfv_      = getCfv_(gamma_, sigma_, PcOverPe_)
            cf_       = getCf_(cfv_, PcOverPa_, sigma_)
            diff_     = (check_ / ambPress_) * 100.0
            if diff_ < 5:
                if flag_ == 0:
                    lineOfMaxCfsX_.append(sigma_)
                    lineOfMaxCfsY_.append(cf_)
                    flag_ = 1
                else:
                    pass
            sigmaData_.append(sigma_)
            cfData_.append(cf_)
        if PcOverPa_ == 1000000000000:
            dp_ = [sigmaData_, cfData_, "INF"]
        else:
            dp_ = [sigmaData_, cfData_, f"{PcOverPa_}"]
        data_.append(dp_)

    # line of seperation
    sigmaData_ = []
    cfData_    = []
    for index_, sigma_ in enumerate(sigmas_):
        PcOverPa_ = getCfMin_(sigma_)
        sigmaData_.append(sigma_)
        cfData_.append(PcOverPa_)
    dp_ = [sigmaData_, cfData_, "Seperation"]
    data_.append(dp_)

    # truncate data
    for i_, dataSet_ in enumerate(data_):
        popIndices_ = []
        for ii_, dataPoint_ in enumerate(dataSet_[1]):
            if cfBottomLim_ < dataPoint_ < cfTopLim_:
                pass
            else:
                popIndices_.append(ii_)
        for iii_ in sorted(popIndices_, reverse=True):
            dataSet_[0].pop(iii_)
            dataSet_[1].pop(iii_)

    # plot
    fig = plt.figure(8, figsize=[7,7])
    ax = fig.add_subplot(111)
    ax.set_xscale("log")
    ax.set_xlim([1.05, 100.0])
    ax.set_ylim([cfBottomLim_, cfTopLim_])
    for index_, dataSet_ in enumerate(data_):
        ax.plot(dataSet_[0], dataSet_[1], label=dataSet_[2])
        thisLine = ax.get_lines()[index_]
        if dataSet_[2] == "500" or dataSet_[2] == "1000" or dataSet_[2] == "INF":
            labelLine(
                thisLine,
                x=dataSet_[0][int(len(dataSet_[0])*0.85)],
                align=False,
                fontsize=11,
                ha="right",
                va="bottom"
            )
        elif dataSet_[2] == "Seperation":
            labelLine(
                thisLine,
                x=dataSet_[0][int(len(dataSet_[0])*0.2)],
                align=False,
                fontsize=11,
                ha="right",
                va="bottom"
            )
        else:
            labelLine(
                thisLine,
                x=dataSet_[0][int(len(dataSet_[0])/2)],
                align=False,
                fontsize=11,
                ha="right",
                va="bottom"
            )
    ax.plot(lineOfMaxCfsX_, lineOfMaxCfsY_, label="Max CF")
    labelLine(
        ax.get_lines()[-1],
        x=lineOfMaxCfsX_[int(len(lineOfMaxCfsX_)*0.7)],
        align=False,
        fontsize=11,
        ha="right",
        va="top"
    )
    ax.plot(DATA["NOZZLE_AREA_RATIO"], DATA["PLOT_FORCE_COEFFICIENT"], label=f"{ID}", color="k")
    labelLine(
        ax.get_lines()[-1],
        x=max(DATA["NOZZLE_AREA_RATIO"]),
        align=False,
        fontsize=11,
        ha="left",
        va="bottom",
        backgroundcolor="none"
    )
    ax.set_title("Thrust Coefficient")
    ###################

    ###################
    plt.show()
    ###################

if PLOT2_FLAG:
    
    ### Figure One. ###
    ### Grain temperature effect on maximum altitude. ###
    plt.figure(1, figsize=[7,7])
    plt.scatter(1, 9812.8317, color="b", label="30F")
    plt.scatter(2, 10000.6568, color="g", label="70F")
    plt.scatter(3, 10211.0928, color="r", label="120F")
    plt.grid()
    plt.xlim([0, 4])
    plt.tick_params(labelsize=12, axis="both", labelbottom=False)
    plt.ylabel("ALTITUDE, (FT)", fontsize=13)
    plt.legend(fontsize="xx-large", loc="upper left")
    ###################

    ### Figure One. ###
    ### Grain temperature effect on maximum speed. ###
    plt.figure(2, figsize=[7,7])
    plt.scatter(1, 703.709, color="b", label="30F")
    plt.scatter(2, 722.76877, color="g", label="70F")
    plt.scatter(3, 745.4425, color="r", label="120F")
    plt.grid()
    plt.xlim([0, 4])
    plt.tick_params(labelsize=12, axis="both", labelbottom=False)
    plt.ylabel("VELOCITY, (FT/S)", fontsize=13)
    plt.legend(fontsize="xx-large", loc="upper left")
    ###################

    ### Figure One. ###
    ### Grain temperature effect on maximum acceleration. ###
    plt.figure(3, figsize=[7,7])
    plt.scatter(1, 6.2963, color="b", label="30F")
    plt.scatter(2, 6.7964, color="g", label="70F")
    plt.scatter(3, 7.4664, color="r", label="120F")
    plt.grid()
    plt.xlim([0, 4])
    plt.tick_params(labelsize=12, axis="both", labelbottom=False)
    plt.ylabel("ACCEL, (Gs)", fontsize=13)
    plt.legend(fontsize="xx-large", loc="upper left")
    ###################

    ###################
    plt.show()
    ###################


































