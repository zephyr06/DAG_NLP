import argparse
import os

parser = argparse.ArgumentParser()
parser.add_argument('--folder', type=str, default="build",
                    help='build or release')

args = parser.parse_args()
folder = args.folder
os.system("python ../CompareWithBaseline/edit_yaml.py --entry debugMode --value 0")

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'



path = "../"+folder+"/tests"
files=os.listdir(path)
files=sorted(files)
count=0
for filename in files:
    if filename[:4]=="test":
        print(bcolors.OKCYAN +"Executing "+filename+ bcolors.ENDC)
        os.system(path+"/"+filename)
        print(bcolors.OKCYAN +"finish execute executable "+filename+ bcolors.ENDC)
        print("")
        count+=1

print(bcolors.OKCYAN +"Finish running ", count, "test files"+ bcolors.ENDC)