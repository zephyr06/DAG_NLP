import argparse
import os
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
import yaml
import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
# from sklearn.datasets.samples_generator import make_blobs
from sklearn.cluster import KMeans
def get_task_set_name():
    copy_file_name = "parameters_copy.yaml"
    with open("../sources/parameters.yaml") as file:
        lines = file.readlines()
        file_to_write = open( copy_file_name ,"w+")
        for line in lines[1:]:
            file_to_write.write(line)
        file_to_write.close()
    with open(copy_file_name) as file:
        prime_service = yaml.safe_load(file)
        return prime_service['testDataSetName']

def get_stat_file():
    test_task_set_name = get_task_set_name()
    return "../release/RunSingleFile/"+test_task_set_name+"_Stat.txt"

def read_stat(file):
    file_name = get_stat_file()
    file = open(file_name)
    Lines = file.readlines()

    data=[]
    for line in Lines:
        data.append(float(line))
    return data

def k_means(data, ncluster=8):
    kmeans = KMeans(n_clusters=ncluster, init='k-means++', max_iter=300, n_init=10, random_state=0)
    data_reshape = np.array(data).reshape((-1,1))
    pred_y = kmeans.fit_predict(data_reshape)
    # print(kmeans.cluster_centers_)
    plt.scatter(kmeans.cluster_centers_[:], np.ones(ncluster), s=300, c='red')
    return kmeans.cluster_centers_

def draw_figure(data):
    data = read_stat(get_stat_file())
    print("Min data: " + str(min(data)))
    # plt.plot(data,'x')
    # plt.scatter(data)
    # plt.hist(data,bins=20)
    sns.histplot(data,  bins=400, kde=True) # bins=1200*3,
    plt.show()





if __name__=="__main__":
    data = read_stat(get_stat_file())
    # plt.plot(data,'x')
    # plt.show()
    center = k_means(data)
    data2 = [round(x) for x in data]
    print("Unique data: " + str(len(np.unique(data2))))
    print(sorted(center, key=float))
    draw_figure(data)