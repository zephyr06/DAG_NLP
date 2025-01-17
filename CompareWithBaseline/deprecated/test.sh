#!/bin/bash

# ***************************************************
# ************** Adjust settings there **************
title="RTDA2CoresPerformance"
MinTaskNumber=3
MaxTaskNumber=10
## no separator '/' at the end of the path
#ROOT_PATH="/home/zephyr/Programming/DAG_NLP" 
ROOT_PATH="/home/dong/workspace/DAG_NLP"
RESULTS_PATH="$ROOT_PATH/TaskData/dagTasks"
# methods_result_dir_name=( "Initial_Res" "OptOrder_Res" "Verucchi_Res" "NLP_Res" )
methods_result_dir_name=( "Initial_Res" "OptOrder_Res" "Verucchi_Res" )
OPTIMIZE_TIME_LIMIT=20
OPTIMIZE_TIME_LIMIT=20
coreNumberAva=2
keep_current_result=False
## setting for generating task sets
TaskSetType=1
taskSetNumber=20
randomSeed=-1 # negative means time seed
# ***************************************************
# ***************************************************

TaskNumberArray=(3 4 5 6 7 8 9 10 15 20 25 30)

# for num in ${TaskNumberArray[@]} ; do
#   echo $num
# done
echo ${TaskNumberArray}
echo ${TaskNumberArray[@]}