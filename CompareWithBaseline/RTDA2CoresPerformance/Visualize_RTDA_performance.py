import copy
import sys
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
import numpy as np
import argparse

sys.path.append("./")


def read_data(taskNumberList):
    all_data_dict = {}
    for method in methods_name:
        folderName = method + "_Res"
        taskNumber_result_dict = {}
        for task_number in taskNumberList:
            file_path = result_file_path + "/" + \
                folderName + "/N" + str(task_number) + ".txt"
            file = open(file_path, "r")
            lines = file.readlines()
            schedulable = []
            objective = []
            time = []
            for result_index in range(int(len(lines) / 3)):
                schedulable.append(float(lines[0 + result_index * 3]))
                objective.append(float(lines[1 + result_index * 3]))
                time.append(float(lines[2 + result_index * 3]))
            file.close()
            result_dict = {"Schedulable": schedulable,
                           "Objective": objective, "Time": time}
            taskNumber_result_dict[task_number] = result_dict
        all_data_dict[method] = taskNumber_result_dict
    return all_data_dict


def Average(lst):
    if len(lst) < 1:
        return -1
    return sum(lst) / len(lst)


def extract_average_data_2d(all_data_dict, taskNumberList, target_obj):
    data_2d = []
    for method in methods_name:
        target_data = []
        for task_number in taskNumberList:
            raw_data = all_data_dict[method][task_number][target_obj]
            target_data.append(Average(raw_data))
        data_2d.append(target_data)
    return data_2d


def normalize_data(data_2d, reference_method="Verucchi"):
    normalized_data = copy.deepcopy(data_2d)
    reference_method_idx = 0
    for i in range(len(methods_name)):
        if (methods_name[i] == reference_method):
            reference_method_idx = i
            break
    for i in range(len(methods_name)):
        if (i != reference_method_idx):
            for j in range(len(normalized_data[i])):
                normalized_data[i][j] /= (
                    normalized_data[reference_method_idx][j] + 0.0)
                normalized_data[i][j] *= 100.0
                # normalized_data[i][j] = min(1.0, normalized_data[i][j])
    for j in range(len(normalized_data[reference_method_idx])):
        normalized_data[reference_method_idx][j] = 100.0

    return normalized_data


def plot_figure(data_to_plot, taskNumberList, ylabel_name):
    dataset_pd = pd.DataFrame()
    optimizer_name = list(methods_name)
    dataset_pd.insert(0, "index", taskNumberList)
    for i in range(len(data_to_plot)):
        dataset_pd.insert(0, optimizer_name[i], data_to_plot[i])
        splot = sns.lineplot(data=dataset_pd, x="index", y=optimizer_name[i], marker=marker_list[i % len(marker_list)],
                             color=color_list[i % len(color_list)], markersize=8)  # , alpha=0.8)
    splot.set(xlabel="Task Number", ylabel=ylabel_name)
    # splot.set_ylim([0.95, 2.0])
    # splot.set_ylim(1e-4, 1e3)
    l_bound = minTaskNumber - 1
    r_bound = maxTaskNumber + 1
    if (l_bound % 2 == 1):
        l_bound -= 1
    if (r_bound % 2 == 0):
        r_bound += 1
    plt.xticks(range(l_bound, r_bound, 2))
    plt.legend(labels=optimizer_name)
    plt.grid(linestyle="--")
    plt.savefig(ylabel_name + "_" + title + ".pdf", format='pdf')
    plt.show(block=False)
    plt.pause(1)
    if (ylabel_name == "Time"):
        splot.set(yscale="log")
        plt.savefig("Time_log_scale_" + title + ".pdf", format='pdf')
        plt.show(block=False)
        plt.pause(1)
    plt.clf()


parser = argparse.ArgumentParser()
parser.add_argument('--minTaskNumber', type=int, default=3,
                    help='Nmin')
parser.add_argument('--maxTaskNumber', type=int, default=10,
                    help='Nmax')
parser.add_argument('--title', type=str, default="RTDA2CoresPerformance",
                    help='tilte in produced figure')
parser.add_argument('--result_file_path', type=str, default="/home/dong/workspace/DAG_NLP/CompareWithBaseline/RTDA2CoresPerformance",
                    help='top directory of result files')
# parser.add_argument('--result_file_path', type=str, default="/home/zephyr/Programming/DAG_NLP/CompareWithBaseline/RTDA2CoresPerformance",
#                     help='project root path')
parser.add_argument('--taskNumberList', type=int, nargs='+', default=[],  # [3, 4, 5, 6, 7, 8, 9, 10, 15, 20, 25, 30],
                    help='The actual task number list in the result files')

args = parser.parse_args()
minTaskNumber = args.minTaskNumber
maxTaskNumber = args.maxTaskNumber
taskNumberList = args.taskNumberList
if (len(taskNumberList) > 0):
    minTaskNumber = taskNumberList[0]
    maxTaskNumber = taskNumberList[-1]
else:
    taskNumberList = list(range(minTaskNumber, maxTaskNumber+1))
title = args.title
result_file_path = args.result_file_path
methods_name = ("Initial", "OrderOptWithoutScheudleOpt", "Verucchi",
                "OrderOpt")
marker_list = ["o", "s", "D", "v"]  #
color_list = ["#0084DB", "limegreen", "y", "r"]  #

if __name__ == "__main__":
    all_data_dict = read_data(taskNumberList)

    schedulable_data_2d = extract_average_data_2d(
        all_data_dict, taskNumberList, "Schedulable")
    objective_data_2d = extract_average_data_2d(
        all_data_dict, taskNumberList, "Objective")
    time_data_2d = extract_average_data_2d(
        all_data_dict, taskNumberList, "Time")
    normalized_objective_data_2d = normalize_data(objective_data_2d)

    plot_figure(schedulable_data_2d, taskNumberList, "Schedulable Ratio")
    plot_figure(objective_data_2d, taskNumberList, "RTDA")
    plot_figure(time_data_2d, taskNumberList, "Time")
    plot_figure(normalized_objective_data_2d,
                taskNumberList, "Normalized RTDA %")
