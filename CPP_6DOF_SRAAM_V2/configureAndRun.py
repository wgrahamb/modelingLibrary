import subprocess

ballistic = 0 # Boolean.
INTEGRATION_METHOD = 0 # 0 = Euler, 1 = RK2, 2 = RK4.
phi = 0 # Degrees.
theta = 45 # Degrees.
psi = 0 # Degrees.
posE = 0 # Meters.
posN = 0 # Meters.
posU = 0 # Meters.
tgtE = 3000 # Meters.
tgtN = 2000 # Meters.
tgtU = 3000 # Meters.
LogData = 1 # Boolean.
ConsoleReport = 1 # Boolean.
ID = "missileEuler"

InputString = f"{ballistic} {INTEGRATION_METHOD} {phi} {theta} \
{psi} {posE} {posN} {posU} {tgtE} {tgtN} {tgtU} {LogData} {ConsoleReport} {ID}\n"
InputFile = r"CPP_6DOF_SRAAM_V2/input/input.txt"
with open(InputFile, "w") as f:
	f.writelines(InputString)
	f.close()

process = subprocess.Popen(["./CPP_6DOF_SRAAM_V2/build/missileModel"])
process.wait()