#!/bin/bash

# ***************************************************
# ************** Adjust settings there **************
title="RTDASingleCorePerformance"
MinTaskNumber=3
MaxTaskNumber=10
#ROOT_PATH="/home/zephyr/Programming/DAG_NLP/"
ROOT_PATH="/home/dong/workspace/DAG_NLP/"
kepp_current_result=False
initial_file_name="Initial_Res"
order_optimization_file_name="OptOrder_Res"
verucchi_file_name="Verucchi_Res"
wang_21rtssic_file_name="NLP_Res"
# ***************************************************
# ***************************************************

# clear buffer file content
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

cp parameters.yaml $ROOT_PATH/sources/parameters.yaml
cd $ROOT_PATH/CompareWithBaseline

python edit_yaml.py --entry "coreNumberAva" --value 1
python edit_yaml.py --entry "TaskSetType" --value 1
python edit_yaml.py --entry "makeProgressTimeLimit" --value 10
python edit_yaml.py --entry "kVerucchiTimeLimit" --value 10

perform_optimization() {
	# Optimize energy consumption
	cd $ROOT_PATH/release
	make -j8
	./tests/tBatch1
	cd $ROOT_PATH/CompareWithBaseline
	sleep 1
}


for (( jobNumber=$MinTaskNumber; jobNumber<=$MaxTaskNumber; jobNumber++ ))
do
	# generate task set
	
	echo "$title iteration is: $jobNumber"
	# initial, order optimization, verucchi
	python edit_yaml.py --entry "weightSF_factor" --value 0
	perform_optimization
	# wang_21rtssIC
	python edit_yaml.py --entry "weightSF_factor" --value 1
	perform_optimization
	
done

# visualize the result
# cd $ROOT_PATH/CompareWithBaseline/$title
# python $ROOT_PATH/CompareWithBaseline/$title/Visualize_performance.py  --minTaskNumber $MinTaskNumber --title $title  --maxTaskNumber $MaxTaskNumber --data_source "EnergySaveRatio"
# python $ROOT_PATH/CompareWithBaseline/$title/Visualize_performance.py  --minTaskNumber $MinTaskNumber --title $title  --maxTaskNumber $MaxTaskNumber --data_source "Time"
# python $ROOT_PATH/CompareWithBaseline/$title/Visualize_performance.py  --minTaskNumber $MinTaskNumber --title $title  --maxTaskNumber $MaxTaskNumber --data_source "RTA"
