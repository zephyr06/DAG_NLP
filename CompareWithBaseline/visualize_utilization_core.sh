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
> ResultFiles/utilization.txt
for i in $(seq 0.1 0.1 0.4) 
do
	for core in {1..8}
	do
		echo "$title iteration is: $core"
		cd ../release
		./tests/GenerateTaskSet --taskSetType 2 --aveUtilization $i --taskSetNumber 10 --NumberOfProcessor $core --N 5 --taskType 1
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
# visualize the result
# python Visualize_average_speed.py --baseline "DAG_RM"
