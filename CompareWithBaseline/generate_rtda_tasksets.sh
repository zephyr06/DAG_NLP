#!/bin/bash

# ***************************************************
# ************** Adjust settings there **************
title="RTDATasksets"
MinTaskNumber=3
MaxTaskNumber=30
TaskNumberArray=(3 4 5 6 7 8 9 10 15 20 25 30)
## no separator '/' at the end of the path
#ROOT_PATH="/home/zephyr/Programming/DAG_NLP" 
ROOT_PATH="/home/dong/workspace/DAG_NLP"
# ROOT_PATH="/home/dong/workspace/DAG_batch_test/rtda_test/DAG_NLP" # for final batch test
RESULTS_PATH="$ROOT_PATH/TaskData/dagTasks" # tBatch1's result path
TASKSETS_PATH="$ROOT_PATH/CompareWithBaseline/TasksetsForRTAS2023/$title" # tBatch1's result path
coreNumberAva=2
## setting for generating task sets
taskSetType=3
taskSetNumber=500
randomSeed=-1 # negative means time seed
# ***************************************************
# ***************************************************

if [[ -d $TASKSETS_PATH ]]; then 
  echo "Already exist $TASKSETS_PATH" 
  echo "exit..."
  exit
fi

mkdir -p $TASKSETS_PATH

# set parameters, backup parameters and scripts parameters
cp RTDA2CoresPerformance/parameters.yaml $ROOT_PATH/sources/parameters.yaml

# major parameter
python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "considerSensorFusion" --value 0

# python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "debugMode" --value 1
python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "coreNumberAva" --value $coreNumberAva
python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "TaskSetType" --value $taskSetType

# build the project
if [[ ! -d $ROOT_PATH/release ]]; then ./build_release_target.sh; fi

# for (( jobNumber=$MinTaskNumber; jobNumber<=$MaxTaskNumber; jobNumber++ ))
for jobNumber in ${TaskNumberArray[@]}
do
	# generate task set
  $ROOT_PATH/release/tests/GenerateTaskSet --N $jobNumber --taskSetType $taskSetType \
    --taskSetNumber $taskSetNumber --NumberOfProcessor $coreNumberAva \
    --randomSeed $randomSeed --useRandomUtilization 1
	
	taskset_folder_name="N$jobNumber"
  
  if [[ ! -d $TASKSETS_PATH/$taskset_folder_name ]]; then mkdir -p $TASKSETS_PATH/$taskset_folder_name; fi
  cp $RESULTS_PATH/*.csv $TASKSETS_PATH/$taskset_folder_name/
done