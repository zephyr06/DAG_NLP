#!/bin/bash

# ***************************************************
# ************** Adjust settings there **************
title="RTDASingleCorePerformance"
MinTaskNumber=3
MaxTaskNumber=3
## no separator '/' at the end of the path
#ROOT_PATH="/home/zephyr/Programming/DAG_NLP" 
ROOT_PATH="/home/dong/workspace/DAG_NLP"
RESULTS_PATH="$ROOT_PATH/TaskData/dagTasks"
keep_current_result=1
methods_result_file_name=( "Initial_Res" "OptOrder_Res" "Verucchi_Res" "NLP_Res" )
coreNumberAva=1
TaskSetType=1
makeProgressTimeLimit=10
kVerucchiTimeLimit=10
# ***************************************************
# ***************************************************

$ROOT_PATH/release/tests/GenerateTaskSet --N 10 --NumberOfProcessor 3