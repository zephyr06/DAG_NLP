#!/bin/bash

methods_result_dir_name=( "Initial_Res" "Verucchi20_Res" "TOM_Res" "TOM_Res_LoopCount" "TOM_Fast_Res" "TOM_Fast_Res_LoopCount" "TOM_FastLP_Res" "TOM_FastLP_Res_LoopCount")
TaskNumberArray=(3 4 5 6 7 8 9 10 15 20 25 30)

for jobNumber in ${TaskNumberArray[@]}
do
    taskset_folder_name="N$jobNumber"
    taskset_result_summary_file_name="N$jobNumber.txt"
    for dir_name in ${methods_result_dir_name[@]}; do
        rm $dir_name/$taskset_result_summary_file_name
        cat $dir_name/$taskset_folder_name/*.txt >> $dir_name/$taskset_result_summary_file_name
    done
done
