#!/usr/bin/bash

task_number_list=( 19 13 12 11 10 21 20 19 18 17 16 15 14 13 12 11 10 9 8 7 6 5 )
files_per_task_list=( 25 25 25 25 25 50 50 50 50 50 50 50 50 50 50 50 50 50 50 50 50 50 )
TOTAL_TASK_NUMBER=4 # the max index that start from 0

MinFileIndex=0
MaxFileIndex=999


perform_optimization() {
	task_number=$1
	files_per_node=$2
	for file_index in $(seq $MinFileIndex $files_per_node $MaxFileIndex); do
		end_index=$((file_index + files_per_node))
		output_file_name=log/BatchDA_${task_number}_${file_index}_${end_index}-$(date +"%Y%m%d%H%M%S").out
		
		echo "Processing N$1: file_index=$file_index:$end_index"
		echo "sbatch -J BatchDA_${task_number}_${file_index}_${end_index} --output=${output_file_name} SBatchDA.sh $1 $file_index $end_index"
		sbatch -J BatchDA_${task_number}_${file_index}_${end_index} --output=${output_file_name} SBatchDA.sh $1 $file_index $end_index
	done
	wait
}


ROOT_PATH=/projects/rtss_let/dong/DAG_NLP
cd $ROOT_PATH/Experiment_ARC/dong/single_task
mkdir log

for task_number_index in $(seq 0 1 $TOTAL_TASK_NUMBER); do
	echo ${task_number_list[task_number_index]} ${files_per_task_list[task_number_index]}
	echo "Processing N=${task_number_list[task_number_index]}:"
	perform_optimization ${task_number_list[task_number_index]} ${files_per_task_list[task_number_index]}
done
