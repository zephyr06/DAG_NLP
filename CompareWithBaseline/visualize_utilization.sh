#!/usr/bin/bash

title="utilization"

# clear buffer file content
> ResultFiles/utilization.txt

cd ../release
make -j8
cd ../CompareWithBaseline
for i in $(seq 0.1 0.1 0.9) 
do
	echo "$title iteration is: $i"
	
	cd ../release
	./tests/GenerateTaskSet --taskSetType 2 --totalUtilization $i --taskSetNumber 500 --NumberOfProcessor 1 --N 5 --taskType 1
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

cd ../CompareWithBaseline
cp ResultFiles/utilization.txt ResultFiles/util_core.txt
# visualize the result
python Visualize_util.py --baseline "RM"
