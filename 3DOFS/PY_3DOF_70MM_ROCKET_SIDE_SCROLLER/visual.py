import matplotlib.pyplot as plt
import pandas as pd
import matPlotLibColors as mc
import matplotlib
matplotlib.use('WebAgg')

f = open("3DOFS/PY_3DOF_70MM_ROCKET_SIDE_SCROLLER/data/log.txt", "r")
df = pd.read_csv(f, delim_whitespace=True)

fig = plt.figure()

startIndex = 0
stopIndex = -1

colors = mc.matPlotLibColors()

ax1 = fig.add_subplot(221)
ax1.plot(df.iloc[startIndex:stopIndex]["RNG"], df.iloc[startIndex:stopIndex]["ALT"], \
	color=colors.pop(0))
ax1.set_title("TRAJECTORY, FEET")

ax2 = fig.add_subplot(222)
ax2.plot(df.iloc[startIndex:stopIndex]["TOF"], df.iloc[startIndex:stopIndex]["RATE"], \
	color=colors.pop(0))
ax2.set_xlabel("TOF")
ax2.set_ylabel("RATE, RADS PER SEC")

ax3 = fig.add_subplot(223)
ax3.plot(df.iloc[startIndex:stopIndex]["TOF"], df.iloc[startIndex:stopIndex]["ALPHA"], \
	color=colors.pop(0))
ax3.set_xlabel("TOF")
ax3.set_ylabel("ALPHA, RADS")

ax4 = fig.add_subplot(224)
ax4.plot(df.iloc[startIndex:stopIndex]["TOF"], df.iloc[startIndex:stopIndex]["WDOT"], \
	color=colors.pop(0))
ax4.set_xlabel("TOF")
ax4.set_ylabel("WDOT, FT PER S^2")

plt.show()



















