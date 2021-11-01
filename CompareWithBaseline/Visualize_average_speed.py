import argparse

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
# from utils_visualize import *
from scipy.optimize import curve_fit


def Read_txt_file_2d(path):
    """ read txt files, and return a 2D list, each element contains MORE THAN one number"""
    file = open(path, 'r')
    lines = file.readlines()
    res = [[],[]]
    i=0
    for i, line in enumerate(lines):
        if(i%2==0):
            res[0].append(float(line[:-1]))
        else:
            res[1].append(float(line[:-1]))
    return np.array(res)

parser = argparse.ArgumentParser()
parser.add_argument('--minTaskNumber', type=int, default=3,
                    help='Nmin')
parser.add_argument('--maxTaskNumber', type=int, default=8,
                    help='Nmax')
parser.add_argument('--baseline', type=str, default="SA2",
                    help='baseline')
parser.add_argument('--ylim', type=float, default=1e2,
                    help='ylim')
args = parser.parse_args()
minTaskNumber = args.minTaskNumber
maxTaskNumber = args.maxTaskNumber
baseline = args.baseline
ylim=args.ylim


# path = "ResultFiles/time_record.txt"
path = "ResultFiles/time_task_number.txt"
data_2d = Read_txt_file_2d(path)
task_number_seq = range(minTaskNumber, maxTaskNumber + 1)
coeff=np.polyfit(task_number_seq, data_2d[1,:],2)
data_simu=[]
for n in task_number_seq:
    data_simu.append(np.polyval(coeff, n))

df=pd.DataFrame({"index":task_number_seq,"NLP":data_2d[1,:], baseline: data_2d[0,:], "simu": data_simu})



splot = sns.lineplot(data=df, x="index", y="NLP",label="NLP",  marker="*", markersize=12, color="#0084DB")
splot = sns.lineplot(data=df, x="index", y=baseline, label=baseline, marker="o", markersize=8, color="limegreen")
# splot = sns.lineplot(data=df, x="index", y="simu", linestyle="--", color="r", label="Quadratic fitting")
# plt.legend(labels=["NLP",baseline])
plt.grid(linestyle="--")
# splot.set(yscale="log")
# plt.grid(linestyle="--")
splot.set(xlabel="Task Number", ylabel="Runt-Time (seconds)")
# splot.set_xlim(4, None)
splot.set_ylim(None, 6)
plt.savefig("Compare_run_time" +baseline+ ".pdf", format='pdf')
plt.savefig("Compare_run_time" +baseline+ ".png", format='png')
plt.show(block=False)
plt.pause(3)
