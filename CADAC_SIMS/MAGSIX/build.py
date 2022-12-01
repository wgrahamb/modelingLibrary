import os
import shutil

try:
	shutil.rmtree("CADAC_SIMS/MAGSIX/build")
except:
	print("No existing build directory. Creating.")
os.mkdir("CADAC_SIMS/MAGSIX/build")
os.chdir("CADAC_SIMS/MAGSIX/build")
os.system("cmake ../")
os.system("make")