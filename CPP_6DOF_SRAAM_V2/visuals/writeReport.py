import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np
import pandas as pd
import os

pdfFile = PdfPages("report.pdf")
fig = plt.figure(figsize=(20,20))

def plotAndWrite(xs, ys, labels, header):
	ax = fig.add_subplot(111)
	colors = ["r", "b", "g", "cyan"]
	for index, x in enumerate(xs):
		ax.plot(x , ys[index], label=labels[index], color=colors[index])
	plt.xlabel("X") 
	plt.ylabel("Y") 
	plt.title(header)
	plt.legend()
	pdfFile.savefig(fig)
	plt.clf()

directory = "output"
dfs = []

for f in os.listdir(directory):
	path = f"{directory}/{f}"
	df = pd.read_csv(open(r"{}".format(path)), delimiter= " ")
	df.name = f
	dfs.append(df)

headers = []
for dfIndex, df in enumerate(dfs):
	if dfIndex == 0:
		for headerIndex, header in enumerate(df.columns):
			headers.append(header)

for header in headers:
	xs = []
	ys = []
	labels = []
	for dfIndex, df in enumerate(dfs):
		xs.append(list(df.iloc[:]["tof"]))
		ys.append(list(df.iloc[:][f"{header}"]))
		labels.append(df.name)
	plotAndWrite(xs, ys, labels, header)

pdfFile.close()