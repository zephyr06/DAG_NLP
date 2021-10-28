#!/usr/bin/bash

title="utilization"

# clear buffer file content
data_buffer_energy="data_buffer_energy_$title.txt"
> $data_buffer_energy
time_file="time_$title.txt"
> $time_file
dataset="../TaskData/$title"

cd ../release
make -j8
cd ../CompareWithBaseline
for i in $(seq 0.1 0.1 0.9) 
do
	echo "$title iteration is: $jobNumber"
	> ResultFiles/utilization.txt
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
cp ResultFiles/utilization.txt ResultFiles/single_util.txt
# visualize the result
python Visualize_util.py --baseline "RM"
