#!/bin/bash

methods_dir_name=( "Initial_Res" "OrderOpt_Res" "Verucchi_Res" "TOM_Res")
TaskNumberArray=(3 4 5 6 7 8 9 10 15 20 25 30)

for jobNumber in ${TaskNumberArray[@]}
do
    taskset_folder_name="N$jobNumber"
    taskset_result_summary_file_name="N$jobNumber.txt"
    for dir_name in ${methods_dir_name[@]}; do
        rm $dir_name/$taskset_result_summary_file_name
        cat $dir_name/$taskset_folder_name/*.txt >> $dir_name/$taskset_result_summary_file_name
    done
done
