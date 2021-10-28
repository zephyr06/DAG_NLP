import argparse

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
# from utils_visualize import *


def Read_txt_file_3d(path, func, delimiter=','):
    """ read txt files, and return a 2D list, each element contains MORE THAN one number"""
    file = open(path, 'r')
    lines = file.readlines()
    res = np.zeros((4,4,2))
    for i, line in enumerate(lines):
        value=float(line[:-1])*100
        core_index=int(i/8)
        util_index=int(i%8/2)
        method_index=i%2
        res[core_index, util_index, method_index]=value
    return np.array(res)

parser = argparse.ArgumentParser()
parser.add_argument('--coreNumber', type=int, default=5,
                    help='Nmin')
parser.add_argument('--maxTaskNumber', type=int, default=20,
                    help='Nmax')
parser.add_argument('--taskSetNumber', type=int, default=100,
                    help='taskSetNumber')
parser.add_argument('--baseline', type=str, default="RM_DAG",
                    help='baseline')
parser.add_argument('--ylim', type=float, default=1e2,
                    help='ylim')
args = parser.parse_args()
coreNumber = args.coreNumber
maxTaskNumber = args.maxTaskNumber
taskSetNumber = args.taskSetNumber
baseline = args.baseline
ylim=args.ylim



path = "ResultFiles/utilization.txt"
data_3d = Read_txt_file_3d(path, lambda x: x)
core_seq = np.arange(1,5)
data_dict={"index":core_seq}
name_aft=": "
for i, method in enumerate([baseline, "NLP"]):
    for util in range(4):
        data_dict[method + name_aft+"0."+str(util+1)]=data_3d[:, util, i]
df=pd.DataFrame(data_dict)
# df=pd.DataFrame({"NLP":data_2d[0,:], baseline: data_2d[1,:]})
# df = pd.DataFrame(data=data_2d.T, columns={"NLP", baseline}, index=range(minTaskNumber, maxTaskNumber+1))

# line, ax=plt.subplots()
# ax=sns.lineplot(x="index", y="NLP", data=df)
markers=["o","v","^","s","|","_","+","x"]
color_per_util=["b","limegreen","r","gold"]
marker_index=0
for i, method in enumerate(["NLP", baseline]):
    for util in range(4):
        # if(method==baseline):
        #     continue
        if(method==baseline):
            splot=sns.lineplot(data=df, x="index", y=method + name_aft+"0."+str(util+1),dashes=True,linestyle="dashed",color=color_per_util[util], marker=markers[marker_index], label=method + name_aft+"0."+str(util+1))
        else:
            splot = sns.lineplot(data=df, x="index", y=method + name_aft+"0." + str(util + 1), dashes=True,color=color_per_util[util],
                                 linestyle="solid", marker=markers[marker_index],
                                 label=method + name_aft+"0." + str(util + 1))
        marker_index=marker_index+1

# splot = sns.lineplot(data=df, x="index", y="NLP",  marker="*", markersize=12)
# splot = sns.lineplot(data=df, x="index", y=baseline, marker="o", markersize=8)
# plt.legend(labels=["NLP",baseline])
# plt.legend()
# splot.set(yscale="log")
plt.grid(linestyle="--")
splot.set(xlabel="Core Number", ylabel="Accept rate")
# splot.set_xlim(4, None)
# splot.set_ylim(1e-3, ylim)
plt.savefig("Core_util" +baseline+ ".pdf", format='pdf')
plt.savefig("Core_util" +baseline+ ".png", format='png')
plt.show(block=False)
plt.pause(3)
