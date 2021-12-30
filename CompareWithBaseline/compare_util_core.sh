#!/usr/bin/bash

folder="compare_util_core"
cp $folder/parameters.yaml ../sources/
cp $folder/GenerateRandomTaskset.h ../sources/
mkdir ../TaskData/dagTasks

# clear buffer file content
> ResultFiles/utilization.txt
> ResultFiles/time_task_number.txt

cd ../release
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j12
cd ../CompareWithBaseline
> ResultFiles/utilization.txt
for i in $(seq 0.1 0.1 0.4) 
do
	for core in {1..4}
	do
		echo "$folder iteration is: $core $i"
		cd ../release
		./tests/GenerateTaskSet --taskSetType 2 --aveUtilization $i --taskSetNumber 1000 --NumberOfProcessor $core --N 5 --taskType 1 --deadlineType 0
		cd ../CompareWithBaseline
		python edit_yaml.py --entry "batchTestMethod" --value 0
		cd ../release
		./tests/testBatch1
		cd ../CompareWithBaseline
		python edit_yaml.py --entry "batchTestMethod" --value 1
		cd ../release
		./tests/testBatch1
		sleep 1
	done
done
cd ../CompareWithBaseline
cp ResultFiles/utilization.txt $folder/util_core.txt
# visualize the result
cd $folder
python Visualize_util_core.py 
