import copy
import sys
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
import numpy as np
import argparse

sys.path.append("./")


def read_LoopCount_data(taskNumberList):
    all_data_dict = {}
    for method in methods_name:
        folderName = method + "_Res_LoopCount"
        taskNumber_result_dict = {}
        for task_number in taskNumberList:
            file_path = result_file_path + "/" + \
                folderName + "/N" + str(task_number) + ".txt"
            file = open(file_path, "r")
            lines = file.readlines()
            whileCount = []
            makeProgressCount = []
            statusCount = []
            for result_index in range(int(len(lines) / 3)):
                whileCount.append(float(lines[0 + result_index * 3]))
                makeProgressCount.append(float(lines[1 + result_index * 3]))
                statusCount.append(float(lines[2 + result_index * 3]))
            file.close()
            result_dict = {"WhileCount": whileCount,
                           "MakeProgressCount": makeProgressCount, "StatusCount": statusCount}
            taskNumber_result_dict[task_number] = result_dict
        all_data_dict[method] = taskNumber_result_dict
    return all_data_dict


def average(lst):
    if len(lst) < 1:
        return -1
    return sum(lst) / len(lst)
def average_ratio(numerator_list, denominator_list):
    if len(numerator_list) < 1 or len(denominator_list) < 1:
        return -1
    lst = []
    for i in range(min(len(numerator_list), len(denominator_list))):
        lst.append((numerator_list[i] + 0.0) / denominator_list[i])
    return sum(lst) / len(lst)


def extract_average_data_2d(all_data_dict, taskNumberList, target_obj):
    data_2d = []
    for method in methods_name:
        target_data = []
        for task_number in taskNumberList:
            raw_data = all_data_dict[method][task_number][target_obj]
            if (target_obj == "Schedulable"):
                target_data.append(average(raw_data)*100.0)
            else:
                target_data.append(average(raw_data))
        data_2d.append(target_data)
    return data_2d

def extract_average_ratio(all_data_dict, taskNumberList, numerator_target, denominator_target):
    data_2d = []
    for method in methods_name:
        target_data = []
        for task_number in taskNumberList:
            numerator_raw_data = all_data_dict[method][task_number][numerator_target]
            denominator_raw_data = all_data_dict[method][task_number][denominator_target]
            target_data.append(100.0 * average_ratio(numerator_raw_data, denominator_raw_data))
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


def extract_relative_gap(data_2d, reference_method="Verucchi"):
    relative_gap = copy.deepcopy(data_2d)
    reference_method_idx = 0
    for i in range(len(methods_name)):
        if (methods_name[i] == reference_method):
            reference_method_idx = i
            break
    for i in range(len(methods_name)):
        if (i != reference_method_idx):
            for j in range(len(data_2d[i])):
                relative_gap[i][j] = (
                    data_2d[i][j] - data_2d[reference_method_idx][j]) / (data_2d[reference_method_idx][j] + 0.0)
                relative_gap[i][j] *= 100.0
    for j in range(len(relative_gap[reference_method_idx])):
        relative_gap[reference_method_idx][j] = 0.0
    return relative_gap


def plot_figure(data_to_plot, taskNumberList, ylabel_name):
    dataset_pd = pd.DataFrame()
    dataset_pd.insert(0, "index", taskNumberList)
    optimizer_name = ["OrderOpt", "TOM", "Only1LP"] # used to name pandas data frame
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
    plt.legend(labels=legends_name)
    plt.grid(linestyle="--")
    plt.savefig(ylabel_name + "_" + title + ".pdf", format='pdf')
    plt.show(block=False)
    plt.pause(1)
    if (ylabel_name == "Run-Time (s)"):
        splot.set(yscale="log")
        plt.savefig("Run-Time (s)_log_scale_" + title + ".pdf", format='pdf')
        plt.show(block=False)
        plt.pause(1)
    plt.clf()


parser = argparse.ArgumentParser()
parser.add_argument('--minTaskNumber', type=int, default=3,
                    help='Nmin')
parser.add_argument('--maxTaskNumber', type=int, default=10,
                    help='Nmax')
parser.add_argument('--title', type=str, default="RTDA",
                    help='tilte in produced figure')
parser.add_argument('--result_file_path', type=str, default="/home/dong/workspace/DAG_NLP/CompareWithBaseline/RTAS2023SupplementaryExperiment/BatchResultForRTAS2023SE/RTDA",
                    help='top directory of result files')
# parser.add_argument('--result_file_path', type=str, default="/home/zephyr/Programming/DAG_NLP/CompareWithBaseline/RTDA2CoresPerformance",
#                     help='project root path')
parser.add_argument('--taskNumberList', type=int, nargs='+', default=[3, 4, 5, 6, 7, 8, 9, 10, 15, 20, 25, 30],
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
# methods_name = ["Initial", "OrderOpt", "Verucchi", "TOM"] # used to read result files
# legends_name = ["Initial", "OrderOpt", "Verucchi20", "TOM"] # "Wang21"
methods_name = ["TOM", "TOM_Fast", "TOM_FastLP"] # used to read result files
legends_name = ["TOM", "TOM_Fast", "TOM_FastLP"] # "Wang21"
marker_list = ["v", "s", "*"]  #
color_list = ["r", "limegreen", "#333333"]  #

if __name__ == "__main__":
    all_data_dict = read_LoopCount_data(taskNumberList)

    whileCount_data_2d = extract_average_data_2d(
        all_data_dict, taskNumberList, "WhileCount")
    makeProgressRatio_data_2d = extract_average_ratio(
        all_data_dict, taskNumberList, "MakeProgressCount", "StatusCount")

    plot_figure(whileCount_data_2d, taskNumberList, "Average Number of Iterations")
    plot_figure(makeProgressRatio_data_2d, taskNumberList, "Make Progress Ratio (%)")
