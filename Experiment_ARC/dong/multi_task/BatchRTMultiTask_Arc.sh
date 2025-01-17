#!/usr/bin/bash

task_number_list=( 21 20 19 18 17 16 15 14 13 12 11 10 9 8 7 6 5 )
files_per_task_list=( 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 8 )
time_per_node_list=( 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 ) # in hour
TOTAL_TASK_NUMBER=16 # the max index start from 0

MinFileIndex=0
MaxFileIndex=999


perform_optimization() {
	task_number=$1
	files_per_task=$2
	time_per_node=$3
	output_file_name=log/BatchRT_${task_number}-$(date +"%Y%m%d%H%M%S").out
	echo "sbatch -J BatchRT_${task_number} --nodes=1 --ntasks-per-node=$(((MaxFileIndex+files_per_task)/files_per_task)) --cpus-per-task=1 --time ${time_per_node}:0:0 --output=${output_file_name} SBatchRTMultiTask.sh $1 $MinFileIndex $files_per_task $MaxFileIndex"
	## test in local
	# bash SBatchRTMultiTask.sh $1 $MinFileIndex $files_per_task $MaxFileIndex
	## launc on ARC
	sbatch -J BatchRT_${task_number} --nodes=1 --ntasks-per-node=$(((MaxFileIndex+files_per_task)/files_per_task)) --cpus-per-task=1 --time ${time_per_node}:0:0 --output=${output_file_name} SBatchRTMultiTask.sh $1 $MinFileIndex $files_per_task $MaxFileIndex
}


ROOT_PATH=/projects/rtss_let/dong/DAG_NLP
# ROOT_PATH=/home/dong/workspace/DAG_NLP
cd $ROOT_PATH/Experiment_ARC/dong/multi_task
mkdir log

for task_number_index in $(seq 0 1 $TOTAL_TASK_NUMBER); do
	echo "Processing N=${task_number_list[task_number_index]} with ${files_per_task_list[task_number_index]} files per task:"
	perform_optimization ${task_number_list[task_number_index]} ${files_per_task_list[task_number_index]} ${time_per_node_list[task_number_index]}
done
