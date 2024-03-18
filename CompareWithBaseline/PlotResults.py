from GlobalVariables import *
from Read_ScheduleRes import *
import sys
import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(1, '~/programming/DAG_NLP/CompareWithBaseline')

# if output_file_name is empty, default name settings will be used
def plot_Obj_results(task_set_number_range, method_names, obj, exclude_time_out=False, output_file_name=""):
    dataset_pd_obj, dataset_pd_runtime = ReadOptResultsAllMethod(
        method_names, obj, task_set_number_range, exclude_time_out)

    plt.figure()
    ax = plt.subplot(111)
    for i in range(len(method_names)):
        splot = sns.lineplot(data=dataset_pd_obj, x="index", y=method_names[i], marker=marker_map[method_names[i]],
                             color=color_map[method_names[i]
                                             ], label=baseline_method_labels[method_names[i]],
                             markersize=marker_size_map[method_names[i]])  # , alpha=alpha_list[i])
    
    plt.xlabel("Number of Tasks", fontsize=axis_label_font_size)
    plt.ylabel("Relative gap(%)", fontsize=axis_label_font_size)

    if (obj == "ReactionTime" or obj == "DataAge"):
        splot.set_ylim([-52, 12])
    else:
        splot.set_ylim([-52, 5])
        
    splot.set_xticks([i for i in range(5,22,2)])
    ax.get_legend().remove()

    # ax.set_title(f"{obj}", fontsize=axis_label_font_size)
    
    if (obj == "ReactionTime"):
        plt.ylabel("Relative Gap of Reaction Time (%)", fontsize=axis_label_font_size)
    elif (obj == "DataAge"):
        plt.ylabel("Relative Gap of Data Age (%)", fontsize=axis_label_font_size)
    elif obj == "SensorFusion":
        plt.ylabel("Relative Gap of Time Disparity (%)", fontsize=axis_label_font_size)
        
    splot.set_xticks([i for i in range(5,21,3)])
    
    plt.grid(linestyle="--")
    plt.tight_layout()
    
    if(output_file_name==""):
        plt.savefig(ROOT_CompareWithBaseline_PATH + obj +
                "/Compare_Performance_" + obj + ".pdf", format='pdf')
    else:
        plt.savefig(ROOT_CompareWithBaseline_PATH + obj +
                    "/Compare_Performance_" + output_file_name + ".pdf", format='pdf')
    plt.show(block=False)
    plt.pause(3)

# if output_file_name is empty, default name settings will be used
def plot_Runtime_results(task_set_number_range, method_names, obj, exclude_time_out=False, output_file_name=""):
    dataset_pd_obj, dataset_pd_runtime = ReadOptResultsAllMethod(
        method_names, obj, task_set_number_range, exclude_time_out)
    plt.figure()
    ax = plt.subplot(111)
    for i in range(len(method_names)):
        splot = sns.lineplot(data=dataset_pd_runtime, x="index", y=method_names[i], marker=marker_map[method_names[i]],
                             color=color_map[method_names[i]
                                             ], label=baseline_method_labels[method_names[i]],
                             markersize=marker_size_map[method_names[i]])  # , alpha=alpha_list[i])
    
    plt.xlabel("Number of Tasks", fontsize=axis_label_font_size)
    plt.ylabel("Running Time (Seconds)", fontsize=axis_label_font_size)
    splot.set_ylim([1e-4, 5e3])
    splot.set(yscale="log")

    splot.set_xticks([i for i in range(5,21,3)])
    splot.set_yticks([10**i for i in range(-4,4)])
    
    ax.get_legend().remove()
    
    # ax.set_title(f'{obj} Running Time')
    
    plt.grid(linestyle="--")
    plt.tight_layout()

    if(output_file_name==""):
        plt.savefig(ROOT_CompareWithBaseline_PATH + obj +
                "/Compare_RunTime_" + obj + ".pdf", format='pdf')
    else:
        plt.savefig(ROOT_CompareWithBaseline_PATH + obj +
                "/Compare_RunTime_" + output_file_name + ".pdf", format='pdf')
    plt.show(block=False)
    plt.pause(3)


def draw_RT_results(task_set_number_range):
    method_names = ["InitialMethod", "SimulatedAnnealing", "Verucchi20", "TOM", "TOM_Fast", "TOM_FarReach"]
                    # "TOM_IA", "Verucchi20"]
    plot_Obj_results(task_set_number_range, method_names, "ReactionTime")
    plot_Runtime_results(task_set_number_range, method_names, "ReactionTime")


def draw_DA_results(task_set_number_range):
    method_names = ["InitialMethod", "SimulatedAnnealing", "Verucchi20", "TOM", "TOM_Fast", "TOM_FarReach"]
                    # "TOM_IA", "Verucchi20"]
    plot_Obj_results(task_set_number_range, method_names, "DataAge")
    plot_Runtime_results(task_set_number_range, method_names, "DataAge")


def draw_SF_results(task_set_number_range, exclude_time_out=False):
    method_names = ["InitialMethod", "SimulatedAnnealing", "TOM", "TOM_Fast", "TOM_FarReach"]
    plot_Obj_results(task_set_number_range, method_names,
                     "SensorFusion", exclude_time_out)
    plot_Runtime_results(task_set_number_range, method_names,
                         "SensorFusion", exclude_time_out)

def print_SF_table_results(task_set_number_range, exclude_time_out=False):
    # draw_SF_results(task_set_number_range, exclude_time_out=False)

    method_names = ["InitialMethod", "TOM", "TOM_Fast", "TOM_IA"]
    dataset_pd_obj, dataset_pd_runtime = ReadOptResultsAllMethod(
        method_names, "SensorFusion", task_set_number_range, exclude_time_out)
    pd.set_option("display.precision", 8)
    pd.set_option('display.precision', 3)
    print("######## SF runtime, 'index' is the maximum of source tasks  ########")
    print(dataset_pd_runtime)
    # pd.options.display.float_format = "{:,.2f}".format
    print("######## SF object, 'index' is the maximum of source tasks  ########")
    print(dataset_pd_obj)


    file = open(ROOT_CompareWithBaseline_PATH + "SensorFusion/stat.txt", "w")
    file.write("\n######## SF object, 'index' is the maximum of source tasks  ########\n")
    file.write(dataset_pd_obj.to_string())
    file.write("\n######## SF runtime, 'index' is the maximum of source tasks  ########\n")
    file.write(dataset_pd_runtime.to_string())
    file.close()