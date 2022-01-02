#!/usr/bin/bash

folder="run_time_speed_automotive"

cp parameters.yaml ../../sources/
cp GenerateRandomTaskset.h ../../sources/
mkdir ../../TaskData/dagTasks

cd ..
# clear buffer file content
> ResultFiles/utilization.txt
> ResultFiles/time_task_number.txt

cd ../release
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j12
cd ../CompareWithBaseline
for taskNumber in {3..8}
do
	echo "$title iteration is: $taskNumber"
	
	cd ../release
	./tests/GenerateTaskSet --taskSetType 2 --totalUtilization 0.4 --taskSetNumber 1000 --NumberOfProcessor 2 --taskType 1 --N $taskNumber
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
cp ResultFiles/time_task_number.txt $folder/time_task_number.txt

# visualize the result
python Visualize2Dplot.py --path "$folder/time_task_number.txt" --minTaskNumber 3 --maxTaskNumber 8 --baseline RM --ylabel "Runt-Time (seconds)" --ylim 100 --save_path "$folder"
