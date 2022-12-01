import os
import shutil

try:
	shutil.rmtree("CADAC_SIMS/SRAAM6/build")
except:
	print("No existing build directory. Creating.")
os.mkdir("CADAC_SIMS/SRAAM6/build")
os.chdir("CADAC_SIMS/SRAAM6/build")
os.system("cmake ../")
os.system("make")