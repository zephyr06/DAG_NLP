import argparse

def Average(lst):
    if len(lst) < 1:
        return -1
    return sum(lst) / len(lst)

parser = argparse.ArgumentParser()
parser.add_argument('--file', type=str, default="N3.txt",
                    help='result file path')

args = parser.parse_args()
file_path = args.file

if __name__ == "__main__":
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
    print("Average obj is: ", Average(objective))
