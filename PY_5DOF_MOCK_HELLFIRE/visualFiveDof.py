import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from utility.matPlotLibColors import matPlotLibColors
import matplotlib
# matplotlib.use('WebAgg')

RAD_TO_DEG = 57.2957795130823

# Setup.
f1          = r"PY_5DOF_MOCK_HELLFIRE/data/MOCK_HELLFIRE5DOF.txt"
viewFile    = f1
df          = pd.read_csv(open(f"{viewFile}"), delimiter=" ")
pitchFinsDF = pd.read_csv(open(
	f"PY_5DOF_MOCK_HELLFIRE/data/PITCH_DEFL.txt"), delimiter=" ")
yawFinsDF   = pd.read_csv(open(
	f"PY_5DOF_MOCK_HELLFIRE/data/YAW_DEFL.txt"), delimiter=" ")
guidanceDF  = pd.read_csv(open(
	f"PY_5DOF_MOCK_HELLFIRE/data/GUIDANCE.txt"), delimiter=" ")
startIndex  = 1
stopIndex   = -1
colors      = matPlotLibColors()
fig         = plt.figure()

# Trajectory
scale = True
trajectory = fig.add_subplot(221, projection="3d")
trajectory.view_init(elev=30, azim=135)
trajectory.set_title("TRAJECTORY")
trajectory.set_xlabel("E", labelpad=10)
trajectory.set_ylabel("N", labelpad=10)
trajectory.set_zlabel("U", labelpad=10)
if scale:
	xMin = min(
		list(df.iloc[startIndex:stopIndex]["ENUPOSX"]) + \
		list(guidanceDF.iloc[startIndex:stopIndex]["TGT_POS_X"])
	)
	xMax = max(
		list(df.iloc[startIndex:stopIndex]["ENUPOSX"]) + \
		list(guidanceDF.iloc[startIndex:stopIndex]["TGT_POS_X"])
	)
	yMin = min(
		list(df.iloc[startIndex:stopIndex]["ENUPOSY"]) + \
		list(guidanceDF.iloc[startIndex:stopIndex]["TGT_POS_Y"])
	)
	yMax = max(
		list(df.iloc[startIndex:stopIndex]["ENUPOSY"]) + \
		list(guidanceDF.iloc[startIndex:stopIndex]["TGT_POS_Y"])
	)
	zMin = min(
		list(df.iloc[startIndex:stopIndex]["ENUPOSZ"]) + \
		list(guidanceDF.iloc[startIndex:stopIndex]["TGT_POS_Z"])
	)
	zMax = max(
		list(df.iloc[startIndex:stopIndex]["ENUPOSZ"]) + \
		list(guidanceDF.iloc[startIndex:stopIndex]["TGT_POS_Z"])
	)
	trajectory.set_box_aspect(
		(
			np.ptp([xMin - 1000, xMax + 1000]), 
			np.ptp([yMin - 1000, yMax + 1000]), 
			np.ptp([zMin, zMax + 1000]),
		)
	)
	trajectory.set_xlim([xMin - 1000, xMax + 1000])
	trajectory.set_ylim([yMin - 1000, yMax + 1000])
	trajectory.set_zlim([zMin, zMax + 1000])
trajectory.plot(
	df.iloc[startIndex:stopIndex]["ENUPOSX"],
	df.iloc[startIndex:stopIndex]["ENUPOSY"],
	df.iloc[startIndex:stopIndex]["ENUPOSZ"],
	color=colors.pop(0)
)
if guidanceDF.iloc[startIndex]["TGT_POS_X"] != \
	guidanceDF.iloc[stopIndex]["TGT_POS_X"]:
	trajectory.plot(
		guidanceDF.iloc[startIndex:stopIndex]["TGT_POS_X"],
		guidanceDF.iloc[startIndex:stopIndex]["TGT_POS_Y"],
		guidanceDF.iloc[startIndex:stopIndex]["TGT_POS_Z"],
		label="TARGET",
		color=colors.pop(0)
	)
else:
	trajectory.scatter(
		guidanceDF.iloc[stopIndex]["TGT_POS_X"],
		guidanceDF.iloc[stopIndex]["TGT_POS_Y"],
		guidanceDF.iloc[stopIndex]["TGT_POS_Z"],
		label="TARGET",
		color=colors.pop(0)
	)
trajectory.legend(fontsize="small")

# Pitch.
pitch = fig.add_subplot(222)
pitch.set_title("PITCH")
pitch.set_xlabel("TOF")
pitch.plot(
	df.iloc[startIndex:stopIndex]["TOF"],
	df.iloc[startIndex:stopIndex]["ALPHA"] * RAD_TO_DEG,
	label="ALPHA - DEG",
	color=colors.pop(0)
)
pitch.plot(
	df.iloc[startIndex:stopIndex]["TOF"],
	df.iloc[startIndex:stopIndex]["ENUTHT"] * RAD_TO_DEG,
	label="THETA - DEG",
	color=colors.pop(0)
)
pitch.plot(
	df.iloc[startIndex:stopIndex]["TOF"],
	df.iloc[startIndex:stopIndex]["QRATE"] * RAD_TO_DEG,
	label="THETA DOT - DEG",
	color=colors.pop(0)
)
pitch.plot(
	df.iloc[startIndex:stopIndex]["TOF"],
	df.iloc[startIndex:stopIndex]["WDOT"],
	label="NORMAL ACC - M/S^2",
	color=colors.pop(0)
)
pitch.plot(
	guidanceDF.iloc[startIndex:stopIndex]["TIME"],
	guidanceDF.iloc[startIndex:stopIndex]["NORM_COMM"],
	label="NORM COMM - M/S^2",
	color=colors.pop(0)
)
pitch.legend(fontsize="small")

# Yaw.
yaw = fig.add_subplot(223)
yaw.set_title("YAW")
yaw.set_xlabel("TOF")
yaw.plot(
	df.iloc[startIndex:stopIndex]["TOF"],
	df.iloc[startIndex:stopIndex]["SIDESLIP"] * RAD_TO_DEG,
	label="SIDESLIP - DEG",
	color=colors.pop(0)
)
yaw.plot(
	df.iloc[startIndex:stopIndex]["TOF"],
	df.iloc[startIndex:stopIndex]["ENUPSI"] * RAD_TO_DEG,
	label="PSI - DEG",
	color=colors.pop(0)
)
yaw.plot(
	df.iloc[startIndex:stopIndex]["TOF"],
	df.iloc[startIndex:stopIndex]["RRATE"] * RAD_TO_DEG,
	label="PSI DOT - DEG",
	color=colors.pop(0)
)
yaw.plot(
	df.iloc[startIndex:stopIndex]["TOF"],
	df.iloc[startIndex:stopIndex]["VDOT"],
	label="SIDE ACC - M/S^2",
	color=colors.pop(0)
)
yaw.plot(
	guidanceDF.iloc[startIndex:stopIndex]["TIME"],
	guidanceDF.iloc[startIndex:stopIndex]["SIDE_COMM"],
	label="SIDE COMM - M/S^2",
	color=colors.pop(0)
)
yaw.legend(fontsize="small")

# Fins
fins = fig.add_subplot(224)
fins.set_title("FINS")
fins.set_xlabel("TOF")
fins.set_ylabel("DEGREES")
fins.plot(
	pitchFinsDF.iloc[startIndex:stopIndex]["TIME"],
	pitchFinsDF.iloc[startIndex:stopIndex]["COMMAND"],
	label="PITCH COMMAND",
	color=colors.pop(0)
)
fins.plot(
	pitchFinsDF.iloc[startIndex:stopIndex]["TIME"],
	pitchFinsDF.iloc[startIndex:stopIndex]["DEFL"],
	label="PITCH DEFLECTION",
	color=colors.pop(0)
)
fins.plot(
	yawFinsDF.iloc[startIndex:stopIndex]["TIME"],
	yawFinsDF.iloc[startIndex:stopIndex]["COMMAND"],
	label="YAW COMMAND",
	color=colors.pop(0)
)
fins.plot(
	yawFinsDF.iloc[startIndex:stopIndex]["TIME"],
	yawFinsDF.iloc[startIndex:stopIndex]["DEFL"],
	label="YAW DEFLECTION",
	color=colors.pop(0)
)
fins.legend(fontsize="small")

# Show.
fig.subplots_adjust(top=0.9, bottom=0.1, left=0.1, hspace=0.4)
plt.show()
