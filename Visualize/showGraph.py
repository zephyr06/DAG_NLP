import argparse
import graphviz
import os

def write_csv_to_graphviz(path, destination):
    file = open(path)
    lines = file.readlines()
    file_d = open(destination, 'w')
    file_d.write('digraph G{')

    for line in lines:
        if (line[0] == '*'):
            prev = line[1:].split(',')[0]
            next = line[1:].split(',')[1]
            file_d.write("Node" + str(prev) + " -> " + "Node" + str(next))
    file_d.write('}')


def show_graphviz(path):
    file = open(path)
    lines = file.readlines()
    dot = graphviz.Digraph(comment='The Round Table')

    for line in lines:
        if (line[0] >= '0' and line[0] <= '9'):
            dot.node("Node" + line.split(',')[0])
        if (line[0] == '*'):
            prev = line[1:].split(',')[0]
            next = line[1:].split(',')[1]
            dot.edge("Node" + str(prev), "Node" + str(next))
    print(dot.source)
    dot.render('currGraph.gv', view=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--n', type=int, default="14",
                        help='the number of test file to visualize, default 14 means test_n5_v14.csv')
    parser.add_argument('--folder', type=str, default="TaskData",
                        help='the number of test file to visualize, default 14 means test_n5_v14.csv')
    parser.add_argument('--root', type=str, default="/home/zephyr/Programming/DAG_NLP/",
                        help='the number of test file to visualize, default 14 means test_n5_v14.csv')

    args = parser.parse_args()
    n = args.n
    folder = args.folder
    root = args.root
    path1 = ""
    if folder == "TaskData":
        path1 = root + "TaskData/test_n5_v" + str(n) + ".csv"
    elif folder == "dagTasks":
        strN = str(n)
        # if n < 10:
        #     strN = "00" + str(n)
        # elif n < 100:
        #     strN = "0" + str(n)
        path1 = root + "TaskData/dagTasks/dag-set-" + strN + "-syntheticJobs" + ".csv"
    show_graphviz(path1)
    os.system("gedit "+path1)
