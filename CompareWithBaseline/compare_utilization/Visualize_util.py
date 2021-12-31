import argparse

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
# from utils_visualize import *


def Read_txt_file_2d(path, func, delimiter=','):
    """ read txt files, and return a 2D list, each element contains MORE THAN one number"""
    file = open(path, 'r')
    lines = file.readlines()
    res = [[],[]]
    i=0
    for i, line in enumerate(lines):
        if(i%2==0):
            res[0].append(float(line[:-1])*100)
        else:
            res[1].append(float(line[:-1])*100)
    return np.array(res)

parser = argparse.ArgumentParser()
parser.add_argument('--minTaskNumber', type=int, default=5,
                    help='Nmin')
parser.add_argument('--maxTaskNumber', type=int, default=20,
                    help='Nmax')
parser.add_argument('--taskSetNumber', type=int, default=100,
                    help='taskSetNumber')
parser.add_argument('--baseline', type=str, default="RM",
                    help='baseline')
parser.add_argument('--ylim', type=float, default=65,
                    help='ylim')
args = parser.parse_args()
minTaskNumber = args.minTaskNumber
maxTaskNumber = args.maxTaskNumber
taskSetNumber = args.taskSetNumber
baseline = args.baseline
ylim=args.ylim



path = "util_core.txt"
data_2d = Read_txt_file_2d(path, lambda x: x)
task_number_seq = np.arange(0.1, 1.0, 0.1)*100
df=pd.DataFrame({"index":task_number_seq,"NLP":data_2d[1,:], baseline: data_2d[0,:]})

splot = sns.lineplot(data=df, x="index", y="NLP",  marker="*", markersize=12, color="#0084DB")
splot = sns.lineplot(data=df, x="index", y=baseline, marker="o", markersize=8, color="orange")
plt.legend(labels=["NLP",baseline])
#splot.set(yscale="log")
plt.grid(linestyle="--")
splot.set(xlabel="Overall utilization (%)", ylabel="Accept rate (%)")
# splot.set_xlim(4, None)
splot.set_ylim(None, ylim)
plt.savefig("Compare_util_single_" +baseline+ ".pdf", format='pdf')
# plt.savefig("Compare_util_single_" +baseline+ ".png", format='png')
plt.show(block=False)
plt.pause(3)
