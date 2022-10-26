#!/bin/bash

# ***************************************************
# ************** Adjust settings there **************
title="VerucchiOnlyRTAS2023"
MinTaskNumber=3
MaxTaskNumber=30
TaskNumberArray=(3 4 5 6 7 8 9 10 15 20 25 30)
## no separator '/' at the end of the path
ROOT_PATH="/home/zephyr/Programming/DAG_NLP" 
# ROOT_PATH="/home/dong/workspace/DAG_NLP"
# ROOT_PATH="/home/dong/workspace/DAG_batch_test/rtda_test/DAG_NLP" # for final batch test
RESULTS_PATH="$ROOT_PATH/TaskData/dagTasks" # tBatch1's result path
methods_dir_name=( "Initial_Res" "Verucchi_Res" ) # only do Initial and Verucchi
TASKSETS_PATH="$ROOT_PATH/CompareWithBaseline/TasksetsForRTAS2023/RTDATasksets"
makeProgressTimeLimit=100
kVerucchiTimeLimit=100
kWangRtss21IcNlpTimeLimit=100
coreNumberAva=2
keep_current_result_and_only_plot=0 # if true, will plot result files in $history_result_directory
history_result_directory="$ROOT_PATH/CompareWithBaseline/VerucchiOnlyRTAS2023/" 
## setting for generating task sets
taskSetType=3
taskSetNumber=500
randomSeed=-1 # negative means time seed
# ***************************************************
# ***************************************************

# if [[ $keep_current_result_and_only_plot == 1 || $keep_current_result_and_only_plot == true \
#   || $keep_current_result_and_only_plot == True || $keep_current_result_and_only_plot == TRUE ]]; then
#   echo "Plot from history in directory: $history_result_directory"
#   # visualize history result
#   python $ROOT_PATH/CompareWithBaseline/$title/Visualize_RTDA_performance.py --minTaskNumber $MinTaskNumber \
#     --title $title --maxTaskNumber $MaxTaskNumber --result_file_path $history_result_directory \
#     --taskNumberList ${TaskNumberArray[@]}
#   cp $ROOT_PATH/CompareWithBaseline/$title/*.pdf $ROOT_PATH/CompareWithBaseline/$title/scripts_and_figures_backup/
#   exit
# fi

echo "Clearing all current results."
for dir_name in ${methods_dir_name[@]}; do
  if [[ -d $dir_name ]]; then rm -rf $dir_name; fi
  mkdir $dir_name
done
if [[ -d dagTasks ]]; then rm -rf dagTasks; fi
mkdir dagTasks
if [[ -d scripts_and_figures_backup ]]; then rm -rf scripts_and_figures_backup; fi
mkdir scripts_and_figures_backup

# set parameters, backup parameters and scripts parameters
cp parameters.yaml $ROOT_PATH/sources/parameters.yaml

# major parameter
python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "considerSensorFusion" --value 0

# python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "debugMode" --value 1
python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "coreNumberAva" --value $coreNumberAva
python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "TaskSetType" --value $taskSetType
python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "makeProgressTimeLimit" --value $makeProgressTimeLimit
python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "kVerucchiTimeLimit" --value $kVerucchiTimeLimit
python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "kWangRtss21IcNlpTimeLimit" --value $kWangRtss21IcNlpTimeLimit
cp $ROOT_PATH/sources/parameters.yaml $ROOT_PATH/CompareWithBaseline/$title/scripts_and_figures_backup
cp $ROOT_PATH/CompareWithBaseline/$title/run$title.sh $ROOT_PATH/CompareWithBaseline/$title/scripts_and_figures_backup

# build the project
cd ..
if [[ ! -d $ROOT_PATH/release ]]; then ./build_release_target.sh; fi

perform_optimization() {
  # Optimize energy consumption
  cd $ROOT_PATH/release
  cmake --build . --config Release -- -j 4
  ./tests/tBatchVerucchi # only test verucchi
  cd $ROOT_PATH/CompareWithBaseline/$title
  sleep 1
}

# for (( jobNumber=$MinTaskNumber; jobNumber<=$MaxTaskNumber; jobNumber++ ))
for jobNumber in ${TaskNumberArray[@]}
do
	# generate task set
  # $ROOT_PATH/release/tests/GenerateTaskSet --N $jobNumber --taskSetType $taskSetType \
  #   --taskSetNumber $taskSetNumber --NumberOfProcessor $coreNumberAva \
  #   --randomSeed $randomSeed --useRandomUtilization 1

	taskset_folder_name="N$jobNumber"

  # instead of generate new task set, copy current tasksets into the RESULTS_PATH path
  cd $RESULTS_PATH
  rm *
  cp $TASKSETS_PATH/$taskset_folder_name/* .
  cd $ROOT_PATH/CompareWithBaseline/$title
	
	echo "$title iteration is: $jobNumber"
	# initial, order optimization, verucchi
	perform_optimization
	
  #copy results to corresponding folder
  taskset_result_summary_file_name="N$jobNumber.txt"
  for dir_name in ${methods_dir_name[@]}; do
    cd $dir_name
    if [[ ! -d $taskset_folder_name ]]; then mkdir $taskset_folder_name; fi
    cp $RESULTS_PATH/*$dir_name.txt ./$taskset_folder_name/
    cat ./$taskset_folder_name/* >> $taskset_result_summary_file_name
    cd $ROOT_PATH/CompareWithBaseline/$title
  done
  if [[ ! -d dagTasks/$taskset_folder_name ]]; then mkdir dagTasks/$taskset_folder_name; fi
  cp $RESULTS_PATH/*.csv dagTasks/$taskset_folder_name/
done

# visualize the result
# python $ROOT_PATH/CompareWithBaseline/$title/Visualize_RTDA_performance.py --minTaskNumber $MinTaskNumber \
#   --title $title --maxTaskNumber $MaxTaskNumber --result_file_path $ROOT_PATH/CompareWithBaseline/$title \
#     --taskNumberList ${TaskNumberArray[@]}
# cp $ROOT_PATH/CompareWithBaseline/$title/*.pdf $ROOT_PATH/CompareWithBaseline/$title/scripts_and_figures_backup/