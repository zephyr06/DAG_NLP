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
    font_size = 15
    plt.xlabel("Task Number", fontsize=font_size)
    plt.ylabel("Relative gap(%)", fontsize=font_size)

    # if (obj == "ReactionTime" or obj == "DataAge"):
        # splot.set_ylim([-82, 5])

    # # Shrink current axis's height by 10% at the botom
    # box = ax.get_position()
    # ax.set_position([box.x0, box.y0 + box.height * 0.21,
    #                  box.width, box.height * 0.9])
    # handles, labels = ax.get_legend_handles_labels()
    # handles = np.concatenate((handles[::3],handles[1::3],handles[2::3]),axis=0)
    # labels = np.concatenate((labels[::3],labels[1::3],labels[2::3]),axis=0)
    # if obj == "DataAge":
    #     plt.legend(handles, labels, loc='lower center', bbox_to_anchor=(0.5, -0.39), ncol=3)
    # elif obj == "ReactionTime":
    #     plt.legend(handles, labels, loc='lower center', bbox_to_anchor=(0.5, -0.32), ncol=3)
    # elif obj == "SensorFusion":
    #     plt.legend(handles, labels, loc='lower center', bbox_to_anchor=(0.5, -0.32), ncol=3)
    # else:
    #     plt.legend()
    # ax.get_legend().remove()

    ax.set_title(f"{obj} Objective")
    plt.grid(linestyle="--")
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
    font_size = 15
    plt.xlabel("Task Number", fontsize=font_size)
    plt.ylabel("Running time (seconds)", fontsize=font_size)
    splot.set_ylim([1e-6, 1e3])
    splot.set(yscale="log")

    # # Shrink current axis's height by 10% at the botom
    # box = ax.get_position()
    # ax.set_position([box.x0, box.y0 + box.height * 0.21,
    #                  box.width, box.height * 0.9])
    # handles, labels = ax.get_legend_handles_labels()
    # handles = np.concatenate((handles[::3],handles[1::3],handles[2::3]),axis=0)
    # labels = np.concatenate((labels[::3],labels[1::3],labels[2::3]),axis=0)
    # if obj == "DataAge":
    #     plt.legend(handles, labels, loc='lower center', bbox_to_anchor=(0.5, -0.39), ncol=3)
    # elif obj == "ReactionTime":
    #     plt.legend(handles, labels, loc='lower center', bbox_to_anchor=(0.5, -0.32), ncol=3)
    # elif obj == "SensorFusion":
    #     plt.legend(handles, labels, loc='lower center', bbox_to_anchor=(0.5, -0.32), ncol=3)
    # else:
    #     plt.legend()
    # ax.get_legend().remove()
    
    ax.set_title(f'{obj} Running Time')
    
    plt.grid(linestyle="--")

    if(output_file_name==""):
        plt.savefig(ROOT_CompareWithBaseline_PATH + obj +
                "/Compare_RunTime_" + obj + ".pdf", format='pdf')
    else:
        plt.savefig(ROOT_CompareWithBaseline_PATH + obj +
                "/Compare_RunTime_" + output_file_name + ".pdf", format='pdf')
    plt.show(block=False)
    plt.pause(3)


def draw_RT_results(task_set_number_range):
    method_names = ["InitialMethod", "TOM", "TOM_Fast", "TOM_IA",
                    "Verucchi20"]
    plot_Obj_results(task_set_number_range, method_names, "ReactionTime")
    plot_Runtime_results(task_set_number_range, method_names, "ReactionTime")


def draw_DA_results(task_set_number_range):
    method_names = ["InitialMethod", "TOM", "TOM_Fast", "TOM_IA",
                    "Verucchi20"]
    plot_Obj_results(task_set_number_range, method_names, "DataAge")
    plot_Runtime_results(task_set_number_range, method_names, "DataAge")

def draw_DA_results3Chains(task_set_number_range):
    method_names = ["InitialMethod", "ImplicitCommunication",
                    "TOM_Sort_Offset", "Bardatsch16", "TOM_BF", "TOM_WSkip", "TOM_Sort"]
    plot_Obj_results(task_set_number_range, method_names, "DataAge", output_file_name="DataAge3Chains")
    plot_Runtime_results(task_set_number_range, method_names, "DataAge", output_file_name="DataAge3Chains")



def draw_SF_results(task_set_number_range, exclude_time_out=False):
    method_names = ["InitialMethod", "TOM", "TOM_Fast", "TOM_IA"]
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