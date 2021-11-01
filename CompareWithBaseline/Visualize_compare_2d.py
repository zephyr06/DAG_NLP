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
parser.add_argument('--baseline', type=str, default="SA",
                    help='baseline')
parser.add_argument('--ylim', type=float, default=25,
                    help='ylim')
parser.add_argument('--path', type=str, default="time_record",
                    help='path in ResultFiles')
parser.add_argument('--withFit', type=int, default=1,
                    help='withFit or not')
args = parser.parse_args()
minTaskNumber = args.minTaskNumber
maxTaskNumber = args.maxTaskNumber
baseline = args.baseline
ylim=args.ylim
path=args.path
withFit=args.withFit


path_file = "ResultFiles/"+path+".txt"
# path = "ResultFiles/time_task_number.txt"
data_2d = Read_txt_file_2d(path_file)
if (path == "accept_rate_record"):
    data_2d=data_2d*100.0
task_number_seq = range(minTaskNumber, maxTaskNumber + 1)
coeff=np.polyfit(task_number_seq, data_2d[1,:],2)
data_simu=[]
for n in task_number_seq:
    data_simu.append(np.polyval(coeff, n))

df=pd.DataFrame({"index":task_number_seq,"NLP":data_2d[1,:], baseline: data_2d[0,:], "simu": data_simu})

palette = sns.color_palette("Paired")

splot = sns.lineplot(data=df, x="index", y="NLP",label="NLP",  marker="*", markersize=12, color="#0084DB") # light blue
splot = sns.lineplot(data=df, x="index", y=baseline, label=baseline, marker="o", markersize=8, color="limegreen")
if withFit:
    splot = sns.lineplot(data=df, x="index", y="simu", linestyle="--", color="r", label="Quadratic fitting")

plt.grid(linestyle="--")
splot.set_ylim(None, ylim)
if(path=="time_record"):
    splot.set(xlabel="Task Number", ylabel="Runt-Time (seconds)")
    plt.savefig("Compare_run_time" +baseline+ ".pdf", format='pdf')
    plt.savefig("Compare_run_time" +baseline+ ".png", format='png')
    plt.show(block=False)
    plt.pause(3)
elif (path == "accept_rate_record"):
    splot.set(xlabel="Task Number", ylabel="Accept rate (%)")
    plt.savefig("Compare_accept_rate" + baseline + ".pdf", format='pdf')
    plt.savefig("Compare_accept_rate" + baseline + ".png", format='png')
    plt.show(block=False)
    plt.pause(3)
elif (path == + "accept_rate_record2"):
    splot.set(xlabel="Task Number", ylabel="Accept rate (%)")
    plt.savefig("Compare_accept_rate2" + baseline + ".pdf", format='pdf')
    plt.savefig("Compare_accept_rate2" + baseline + ".png", format='png')
    plt.show(block=False)
    plt.pause(3)
