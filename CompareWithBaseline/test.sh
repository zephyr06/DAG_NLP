#!/bin/bash

# ***************************************************
# ************** Adjust settings there **************
title="RTDASingleCorePerformance"
MinTaskNumber=3
MaxTaskNumber=10
#ROOT_PATH="/home/zephyr/Programming/DAG_NLP/"
ROOT_PATH="/home/dong/workspace/DAG_NLP/"
keep_current_result=1
initial_file_name="Initial_Res"
order_optimization_file_name="OptOrder_Res"
verucchi_file_name="Verucchi_Res"
wang_21rtssic_file_name="NLP_Res"
# ***************************************************
# ***************************************************

if [[ $keep_current_result == 0 || $keep_current_result == false || $keep_current_result == False ]]; then
  echo "Clearing all current results."
  if [[ -d $initial_file_name ]]; then rm -rf $initial_file_name; fi
  if [[ -d $order_optimization_file_name ]]; then rm -rf $order_optimization_file_name; fi
  if [[ -d $verucchi_file_name ]]; then rm -rf $verucchi_file_name; fi
  if [[ -d $wang_21rtssic_file_name ]]; then rm -rf $wang_21rtssic_file_name; fi
  mkdir $initial_file_name $order_optimization_file_name $verucchi_file_name $wang_21rtssic_file_name
else
  if [[ ! -d $initial_file_name ]]; then mkdir $initial_file_name; fi
  if [[ ! -d $order_optimization_file_name ]]; then mkdir $order_optimization_file_name; fi
  if [[ ! -d $verucchi_file_name ]]; then mkdir $verucchi_file_name; fi
  if [[ ! -d $wang_21rtssic_file_name ]]; then mkdir $wang_21rtssic_file_name; fi
fi