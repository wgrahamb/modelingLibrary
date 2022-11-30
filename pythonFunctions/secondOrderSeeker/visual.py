import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation as fa
import matplotlib
matplotlib.use("WebAgg")

df = pd.read_csv("pythonFunctions/secondOrderSeeker/seekerLog.txt", delimiter=" ")
fig = plt.figure()
window = fig.add_subplot(111)
window.set_xlim([-1, 1])
window.set_ylim([-1, 1])
window.set_xlabel("YAW (RADIANS)")
window.set_ylabel("PITCH (RADIANS)")
boreSight = window.scatter([], [], color="g", marker="+", label="BORESIGHT")
seeker = window.scatter([], [], color="b", marker="2", s=200, label="SEEKER ANGULAR OFFSET FROM BORESIGHT")
target = window.scatter([], [], color="r", marker="x", label="TARGET ANGULAR OFFSET FROM BORESIGHT")
title = window.text(0.5, 0.95, "0.0", transform=window.transAxes, ha="center")

def update(frames, data):
	time = data.iloc[frames]["time"]
	title.set_text(f"{time:.3f}")
	boreSight.set_offsets(
		[data.iloc[frames]["boreSightYaw"], data.iloc[frames]["boreSightPitch"]]
	)
	seeker.set_offsets(
		[data.iloc[frames]["seekerYawOffBoreSight"], data.iloc[frames]["seekerPitchOffBoreSight"]]
	)
	target.set_offsets(
		[data.iloc[frames]["targetYawOffBoreSight"], data.iloc[frames]["targetPitchOffBoreSight"]]
	)
	return [boreSight, seeker, target, title,]

animation = fa(
	fig=fig,
	func=update,
	frames=len(df),
	fargs=([df]),
	repeat=True,
	repeat_delay=1500,
	blit=True,
	interval=0.0001
)

window.legend(fontsize="small", loc="lower left")
plt.get_current_fig_manager().full_screen_toggle()
plt.show()
