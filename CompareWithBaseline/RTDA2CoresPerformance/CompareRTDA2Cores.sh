#!/bin/bash

# ***************************************************
# ************** Adjust settings there **************
title="RTDA2CoresPerformance"
MinTaskNumber=3
MaxTaskNumber=20
## no separator '/' at the end of the path
#ROOT_PATH="/home/zephyr/Programming/DAG_NLP" 
ROOT_PATH="/home/dong/workspace/DAG_NLP"
RESULTS_PATH="$ROOT_PATH/TaskData/dagTasks"
methods_dir_name=( "Initial_Res" "OptOrder_Res" "Verucchi_Res" "NLP_Res" )
makeProgressTimeLimit=20
kVerucchiTimeLimit=20
coreNumberAva=2
keep_current_result=False
## setting for generating task sets
TaskSetType=1
taskSetNumber=3
randomSeed=-1 # negative means time seed
# ***************************************************
# ***************************************************

if [[ $keep_current_result == 0 || $keep_current_result == false || $keep_current_result == False ]]; then
  echo "Clearing all current results."
  for dir_name in ${methods_dir_name[@]}; do
    if [[ -d $dir_name ]]; then rm -rf $dir_name; fi
    mkdir $dir_name
  done
else
  for dir_name in ${methods_dir_name[@]}; do
    if [[ ! -d $dir_name ]]; then mkdir $dir_name; fi
  done
fi

cp parameters.yaml $ROOT_PATH/sources/parameters.yaml
# build the project
cd ..
if [[ ! -d $ROOT_PATH/release ]]; then ./build_release_target.sh; fi

python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "coreNumberAva" --value $coreNumberAva
python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "TaskSetType" --value $TaskSetType
python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "makeProgressTimeLimit" --value $makeProgressTimeLimit
python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "kVerucchiTimeLimit" --value $kVerucchiTimeLimit

perform_optimization() {
# Optimize energy consumption
cd $ROOT_PATH/release
make -j8
./tests/tBatch1
cd $ROOT_PATH/CompareWithBaseline/$title
sleep 1
}

for (( jobNumber=$MinTaskNumber; jobNumber<=$MaxTaskNumber; jobNumber++ ))
do
	# generate task set
  $ROOT_PATH/release/tests/GenerateTaskSet --N $jobNumber --taskSetNumber $taskSetNumber --NumberOfProcessor $coreNumberAva --randomSeed $randomSeed
	
	echo "$title iteration is: $jobNumber"
	# initial, order optimization, verucchi
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "weightSF_factor" --value 0
	perform_optimization

	# wang_21rtssIC
	python $ROOT_PATH/CompareWithBaseline/edit_yaml.py --entry "weightSF_factor" --value 1
	perform_optimization
	
  #copy results to corresponding folder
	taskset_folder_name="N$jobNumber"
  taskset_result_summary_file_name="N$jobNumber.txt"
  for dir_name in ${methods_dir_name[@]}; do
    cd $dir_name
    if [[ ! -d $taskset_folder_name ]]; then mkdir $taskset_folder_name; fi
    cp $RESULTS_PATH/*$dir_name.txt ./$taskset_folder_name/
    cat ./$taskset_folder_name/* >> $taskset_result_summary_file_name
    cd $ROOT_PATH/CompareWithBaseline/$title
  done

done

visualize the result
python $ROOT_PATH/CompareWithBaseline/$title/Visualize_performance.py  --minTaskNumber $MinTaskNumber --title $title  --maxTaskNumber $MaxTaskNumber --root_path $ROOT_PATH