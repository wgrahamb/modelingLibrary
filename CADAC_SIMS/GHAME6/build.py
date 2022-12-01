import os
import shutil

try:
	shutil.rmtree("CADAC_SIMS/GHAME6/build")
except:
	print("No existing build directory. Creating.")
os.mkdir("CADAC_SIMS/GHAME6/build")
os.chdir("CADAC_SIMS/GHAME6/build")
os.system("cmake ../")
os.system("make")