import argparse

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns


# from utils_visualize import *


def Read_txt_file_2d(path):
    """ read txt files, and return a 2D list, each element contains MORE THAN one number"""
    file = open(path, 'r')
    lines = file.readlines()
    res = [[], []]
    i = 0
    for i, line in enumerate(lines):
        if (i % 2 == 0):
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
parser.add_argument('--ylabel', type=str, default=1e2,
                    help='Runt-Time (seconds)')
parser.add_argument('--path', type=str, default="ResultFiles/time_record_AM.txt",
                    help='path for file to plot')
parser.add_argument('--withfit', type=int, default=0,
                    help='with quadratic fitting or not')
parser.add_argument('--save_path', type=str, default="run_time_speed_automotive",
                    help='path to save images')
parser.add_argument('--fig_title', type=str, default="Compare_run_time",
                    help='fig_title')

args = parser.parse_args()
minTaskNumber = args.minTaskNumber
maxTaskNumber = args.maxTaskNumber
baseline = args.baseline
path = args.path
ylim = args.ylim
ylabel = args.ylabel
withfit = args.withfit
save_path = args.save_path
fig_title = args.fig_title

# path = "ResultFiles/time_record_AM.txt"
# path = "ResultFiles/time_task_number.txt"
data_2d = Read_txt_file_2d(path)
task_number_seq = range(minTaskNumber, maxTaskNumber + 1)
coeff = np.polyfit(task_number_seq, data_2d[1, :], 2)
data_simu = []
for n in task_number_seq:
    data_simu.append(np.polyval(coeff, n))

df = pd.DataFrame({"index": task_number_seq, "NLP": data_2d[1, :], baseline: data_2d[0, :], "simu": data_simu})

splot = sns.lineplot(data=df, x="index", y="NLP", label="NLP", marker="*", markersize=12, color="#0084DB")
splot = sns.lineplot(data=df, x="index", y=baseline, label=baseline, marker="o", markersize=8, color="limegreen")
if withfit:
    splot = sns.lineplot(data=df, x="index", y="simu", linestyle="--", color="r", label="Quadratic fitting")
# plt.legend(labels=["NLP",baseline])
plt.grid(linestyle="--")
# splot.set(yscale="log")
# plt.grid(linestyle="--")
splot.set(xlabel="Task Number", ylabel=ylabel)
# splot.set_xlim(4, None)
# splot.set_ylim(1e-3, ylim)
plt.savefig(save_path + "/" + fig_title + ".pdf", format='pdf')
# plt.savefig("Compare_run_time" + ".png", format='png')
plt.show(block=False)
plt.pause(3)
