import numpy as np
STD_GRAV = 32.2

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