#!/bin/bash

# ***************************************************
# ************** Adjust settings there **************
title="OrderOptOnlySensorFusionRTAS2023"
MinTaskNumber=3
MaxTaskNumber=30
TaskNumberArray=(3 4 5 6 7 8 9 10 15 20 25 30)
# TaskNumberArray=(6 7 8 9 10)
## no separator '/' at the end of the path
# ROOT_PATH="/home/zephyr/Programming/DAG_NLP" 
ROOT_PATH="/home/dong/workspace/DAG_NLP"
# ROOT_PATH="/home/zephyr/Programming/batch_test_DAG_NLP/VerucchiOnly/N3_10" # for final batch test
RESULTS_PATH="$ROOT_PATH/TaskData/dagTasks" # tBatch's result path
methods_result_dir_name=( "Initial_Res" "OrderOpt_Res" "OrderOpt_Res_LoopCount" ) # only do Initial and OrderOpt
target_method_res_name="OrderOpt_Res"
TASKSETS_PATH="$ROOT_PATH/CompareWithBaseline/TasksetsForRTAS2023/SensorFusionTasksets"
OPTIMIZE_TIME_LIMIT=100
OPTIMIZE_TIME_LIMIT=100
OPTIMIZE_TIME_LIMIT=100
coreNumberAva=2
keep_current_result_and_continue_previous_running=1 # 0 will delete all current results and rerun, 1 will attach results to current result files
## setting for generating task sets
taskSetType=3
taskSetNumber=500
randomSeed=-1 # negative means time seed
# ***************************************************
# ***************************************************

call_the_executable() {
  cd $ROOT_PATH/release
  ./tests/tBatchOrderOpt # only test OrderOpt
  cd $ROOT_PATH/CompareWithBaseline/$title
}

if [[ $keep_current_result_and_continue_previous_running == 1 ]]; then
  for dir_name in ${methods_result_dir_name[@]}; do
    if [[ ! -d $dir_name ]]; then mkdir $dir_name; fi
  done
  if [[ ! -d dagTasks ]]; then mkdir dagTasks; fi
  if [[ ! -d scripts_and_figures_backup ]]; then mkdir scripts_and_figures_backup; fi
else
  echo "Clearing all current results."
  for dir_name in ${methods_result_dir_name[@]}; do
    if [[ -d $dir_name ]]; then rm -rf $dir_name; fi
    mkdir $dir_name
  done
  if [[ -d dagTasks ]]; then rm -rf dagTasks; fi
  mkdir dagTasks
  if [[ -d scripts_and_figures_backup ]]; then rm -rf scripts_and_figures_backup; fi
  mkdir scripts_and_figures_backup
fi
# make directories for results
cd $ROOT_PATH/CompareWithBaseline/$title
for jobNumber in ${TaskNumberArray[@]}; do
	taskset_folder_name="N$jobNumber"
  for dir_name in ${methods_result_dir_name[@]}; do
    cd $dir_name
    if [[ ! -d $taskset_folder_name ]]; then mkdir $taskset_folder_name; fi
    cd ..
  done
  if [[ ! -d dagTasks/$taskset_folder_name ]]; then mkdir dagTasks/$taskset_folder_name; fi
done

# set parameters, backup parameters and scripts parameters
cp parameters.yaml $ROOT_PATH/sources/parameters.yaml

# major parameter
python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "considerSensorFusion" --value 1

# python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "debugMode" --value 1
python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "coreNumberAva" --value $coreNumberAva
python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "TaskSetType" --value $taskSetType
python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "OPTIMIZE_TIME_LIMIT" --value $OPTIMIZE_TIME_LIMIT
python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "OPTIMIZE_TIME_LIMIT" --value $OPTIMIZE_TIME_LIMIT
python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "OPTIMIZE_TIME_LIMIT" --value $OPTIMIZE_TIME_LIMIT
cp $ROOT_PATH/sources/parameters.yaml $ROOT_PATH/CompareWithBaseline/$title/scripts_and_figures_backup
cp $ROOT_PATH/CompareWithBaseline/$title/run$title.sh $ROOT_PATH/CompareWithBaseline/$title/scripts_and_figures_backup
# build the project
cd ..
if [[ ! -d $ROOT_PATH/release ]]; then ./build_release_target.sh; fi

# for (( jobNumber=$MinTaskNumber; jobNumber<=$MaxTaskNumber; jobNumber++ ))
for jobNumber in ${TaskNumberArray[@]}
do
	# generate task set
  # $ROOT_PATH/release/tests/GenerateTaskSet --N $jobNumber --taskSetType $taskSetType \
  #   --taskSetNumber $taskSetNumber --NumberOfProcessor $coreNumberAva \
  #   --randomSeed $randomSeed --useRandomUtilization 1

	echo "$title iteration is: $jobNumber"

  cd $ROOT_PATH/CompareWithBaseline/$title
	taskset_folder_name="N$jobNumber"

  for ((taskset_id = 0 ; taskset_id < taskSetNumber ; taskset_id++)); do
    printf -v padded_id "%03d" $taskset_id

    if [[ ! -e ./$target_method_res_name/$taskset_folder_name/dag-set-$taskset_folder_name-$padded_id-syntheticJobs.csv_$target_method_res_name.txt ]]; then
      # instead of generate new task set, copy current tasksets into the RESULTS_PATH path
      cd $RESULTS_PATH
      rm *
      cp $TASKSETS_PATH/$taskset_folder_name/dag-set-$taskset_folder_name-$padded_id-syntheticJobs.csv .
      cd $ROOT_PATH/CompareWithBaseline/$title
      call_the_executable
      
      #copy results to corresponding folder
      taskset_result_summary_file_name="N$jobNumber.txt"
      for dir_name in ${methods_result_dir_name[@]}; do
        cd $dir_name
        cp $RESULTS_PATH/dag-set-$taskset_folder_name-$padded_id-syntheticJobs.csv_*$dir_name.txt ./$taskset_folder_name/
        # rm $taskset_result_summary_file_name
        cat ./$taskset_folder_name/dag-set-$taskset_folder_name-$padded_id-syntheticJobs.csv_*$dir_name.txt >> $taskset_result_summary_file_name
        cd $ROOT_PATH/CompareWithBaseline/$title
      done
      if [[ ! -d dagTasks/$taskset_folder_name ]]; then mkdir dagTasks/$taskset_folder_name; fi
      cp $RESULTS_PATH/*.csv dagTasks/$taskset_folder_name/
    fi
  done
done
