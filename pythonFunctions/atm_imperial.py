import numpy as np
STD_GRAV = 32.2

def atmosphere(altitude, spd): # feet. ft/s

    if altitude > 32809:
        print("Altitude exceeds 32,809 feet.")
        print("Model ceiling exceeded. Terminating.")
        exit(0)

    def RtoK(T):
        ret = T/1.8
        return ret

    pressure    = None # air pressure in psi
    temperature = None # air temperature in Rankine
    rho         = None # air density in lbm/ft^3
    a           = None # speed of sound in ft/s
    q           = None # dynamic pressure in psi
    g           = None # gravity in ft/s^2
    mach        = None # mach speed, non dimensional
    rad         = 2.0856e+7 # radius of earth in feet

    pressure = \
        4.272981E-14*(altitude**3) + \
        0.000000008060081*(altitude**2) - \
        0.0005482655*(altitude) + \
        14.692410 # psi
    temperature = -1.0 * 0.0036 * altitude + 518.399 # Rankine
    rho = \
        (0.00000000001255)*(altitude**2)- \
        (0.0000019453)*altitude + \
        0.07579 # lbm/ft^3
    q = (0.5 * rho * spd * spd) * (1 / STD_GRAV) * (1.0 / 144.0)
    g = STD_GRAV * ((rad / (rad+altitude)) ** 2)
    a = (np.sqrt(RtoK(temperature) * 1.4 * 286)) * 3.28084
    mach = spd / a

    return pressure, temperature, rho, a, q, g, mach