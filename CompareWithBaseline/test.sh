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
# methods_dir_name=( "Initial_Res" "OptOrder_Res" "Verucchi_Res" "NLP_Res" )
methods_dir_name=( "Initial_Res" "OptOrder_Res" "Verucchi_Res" )
makeProgressTimeLimit=20
kVerucchiTimeLimit=20
coreNumberAva=2
keep_current_result=False
## setting for generating task sets
TaskSetType=1
taskSetNumber=20
randomSeed=-1 # negative means time seed
# ***************************************************
# ***************************************************

for DIR in `find $ROOT_PATH -type d -name backup` ; do
  # echo $DIR
  cd $DIR
  files=`ls`
  for file in $files ; do
    if [[ ${file:(-4):4} != ".tar" ]] ; then
      if [[ ! ${files[*]} =~ $file.tar ]]; then
        echo "Make tar ball of directory: $DIR/$file"
        tar -cf $file.tar $file
      fi
    fi
  done
done