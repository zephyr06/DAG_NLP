#!/usr/bin/bash

title="utilization"

cd ../release
cd ../CompareWithBaseline
> ResultFiles/utilization.txt
for i in $(seq 0.4 0.1 0.4) 
do
	for core in {1..4}
	do
		echo "$title iteration is: $core $i"
		cd ../release
		./tests/GenerateTaskSet --taskSetType 2 --aveUtilization $i --taskSetNumber 1000 --NumberOfProcessor $core --N 5 --taskType 1
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
cp ResultFiles/utilization.txt ResultFiles/util_core.txt
# visualize the result
python Visualize_util_core.py 