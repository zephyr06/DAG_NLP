#!/bin/bash

# ***************************************************
# ************** Adjust settings there **************
title="RTDASingleCorePerformance"
MinTaskNumber=3
MaxTaskNumber=7
## no separator '/' at the end of the path
#ROOT_PATH="/home/zephyr/Programming/DAG_NLP" 
ROOT_PATH="/home/dong/workspace/DAG_NLP"
RESULTS_PATH="$ROOT_PATH/TaskData/dagTasks"
keep_current_result=False
methods_result_file_name=( "Initial_Res" "OptOrder_Res" "Verucchi_Res" "NLP_Res" )
coreNumberAva=1
TaskSetType=1
makeProgressTimeLimit=10
kVerucchiTimeLimit=10
## setting for generating task sets
taskSetNumber=10
randomSeed=-1 # negative means time seed
# ***************************************************
# ***************************************************

if [[ $keep_current_result == 0 || $keep_current_result == false || $keep_current_result == False ]]; then
  echo "Clearing all current results."
  for file_name in ${methods_result_file_name[@]}; do
    if [[ -d $file_name ]]; then rm -rf $file_name; fi
    mkdir $file_name
  done
else
  for file_name in ${methods_result_file_name[@]}; do
    if [[ ! -d $file_name ]]; then mkdir $file_name; fi
  done
fi

cp parameters.yaml $ROOT_PATH/sources/parameters.yaml

# build the project
if [[ ! -d $ROOT_PATH/release ]]; then ./build_release_target.sh; fi

python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "coreNumberAva" --value $coreNumberAva
python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "TaskSetType" --value $TaskSetType
python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "makeProgressTimeLimit" --value $makeProgressTimeLimit
python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "kVerucchiTimeLimit" --value $kVerucchiTimeLimit

perform_optimization() {
# Optimize energy consumption
cd $ROOT_PATH/release
make -j8
./tests/tBatch1
cd $ROOT_PATH/CompareWithBaseline/$title
sleep 1
}

for (( jobNumber=$MinTaskNumber; jobNumber<=$MaxTaskNumber; jobNumber++ ))
do
	# generate task set
  $ROOT_PATH/release/tests/GenerateTaskSet --N $jobNumber --taskSetNumber $taskSetNumber --NumberOfProcessor $coreNumberAva --randomSeed $randomSeed
	
	echo "$title iteration is: $jobNumber"
	# initial, order optimization, verucchi
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "weightSF_factor" --value 0
	perform_optimization

	# wang_21rtssIC
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "weightSF_factor" --value 1
	perform_optimization
	
  #copy results to corresponding folder
	taskset_folder_name="N$jobNumber"
  for file_name in ${methods_result_file_name[@]}; do
    cd $file_name
    if [[ ! -d $taskset_folder_name ]]; then mkdir $taskset_folder_name; fi
    cp $RESULTS_PATH/*$file_name.txt ./$taskset_folder_name/
    cd $ROOT_PATH/CompareWithBaseline/$title
  done

done

# visualize the result
# cd $ROOT_PATH/CompareWithBaseline/$title
# python $ROOT_PATH/CompareWithBaseline/$title/Visualize_performance.py  --minTaskNumber $MinTaskNumber --title $title  --maxTaskNumber $MaxTaskNumber --data_source "EnergySaveRatio"
# python $ROOT_PATH/CompareWithBaseline/$title/Visualize_performance.py  --minTaskNumber $MinTaskNumber --title $title  --maxTaskNumber $MaxTaskNumber --data_source "Time"
# python $ROOT_PATH/CompareWithBaseline/$title/Visualize_performance.py  --minTaskNumber $MinTaskNumber --title $title  --maxTaskNumber $MaxTaskNumber --data_source "RTA"