import os
from GlobalVariables import *


class ScheduleResult:
    def __init__(self, schedulable, obj, runtime, file_path):
        self.schedulable = schedulable
        self.obj = obj
        self.runtime = runtime
        self.file_path = file_path

    def print(self):
        print(str(self.schedulable) + ", " +
              str(self.obj) + ", " + str(self.runtime))


def ReadScheduleResult(file_path):
    file = open(file_path, 'r')
    lines = file.readlines()
    return ScheduleResult(lines[0][lines[0].find(":")+2:], float(lines[1][lines[1].find(":")+2:]), float(lines[2][lines[2].find(":")+2:]), file_path)


def PrintResultVec(res_vec):
    for res in res_vec:
        res.print()


# return 1D array of the results
def ReadOptResults(method_name="TOM_Sort", obj_type="DataAge", task_set_number=5, root_path=ROOT_PATH):
    data_set_folder = root_path + "TaskData/N" + str(task_set_number)
    res_vec = []
    files_all = os.listdir(data_set_folder)
    files_all.sort()
    for filename in files_all:
        file_signature = method_name + "_" + obj_type
        if (filename.find(file_signature) != -1):
            file_path = os.path.join(data_set_folder, filename)
            res_vec.append(ReadScheduleResult(file_path))
    if (len(res_vec) == 0):
        print('\033[93m' + method_name + " not found!")
    return res_vec



# Normalize based on the method called "InitialMethod"
def Normalize(obj_vec, obj_base):
    return (obj_vec-obj_base) / float(obj_base)*100.0
# def Normalize(obj_vec, obj_base):
#     return (obj_vec) / float(obj_base)*100.0


def Average(res_vec, base_vec, obj_type="DataAge", task_num=5, method_name = "TOM", exclude_time_out=False, excluded_table=[]):
    average_obj = 0
    average_runtime = 0
    total_case = len(base_vec)
    timeout_case = 0
    # for res in res_vec:
    for i in range(len(res_vec)):
        if exclude_time_out and excluded_table[i]:
            # print("Exclude one time out case")
            total_case -= 1
            continue
        if (obj_type == "ReactionTime" or obj_type == "DataAge"):
            # if Verucchi has no result, populate the initial solution
            if method_name == 'Verucchi20' and res_vec[i].obj < 0:
                res_vec[i].obj = base_vec[i].obj
                
            if (float(res_vec[i].obj) / base_vec[i].obj > 1.1 and method_name != 'Verucchi20'):
                print("Find an error result! N{}_{}_{}_{}".format(task_num, i, method_name, obj_type))
            average_obj += Normalize(res_vec[i].obj, base_vec[i].obj)
            average_runtime += res_vec[i].runtime
            if res_vec[i].runtime > 1000:
                timeout_case+=1
        elif obj_type=="SensorFusion":
            if (base_vec[i].obj > 0):
                average_obj += Normalize(res_vec[i].obj, base_vec[i].obj)
            else:
                total_case -= 1
            average_runtime += res_vec[i].runtime
            if res_vec[i].runtime > 1000:
                timeout_case+=1
    print("T", task_num, method_name, obj_type, "time out case count:", timeout_case, ", average time is:", average_runtime / total_case)
    return average_obj / total_case, average_runtime / total_case
