#!/usr/bin/bash

folder="compare_SA"

cp parameters.yaml ../../sources/
cp GenerateRandomTaskset.h ../../sources/
mkdir ../TaskData/dagTasks

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
	./tests/GenerateTaskSet --taskSetType 2 --totalUtilization 0.4 --taskSetNumber 10 --NumberOfProcessor 2 --taskType 1 --N $taskNumber
	cd ../CompareWithBaseline
	python edit_yaml.py --entry "batchTestMethod" --value 2
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
cp ResultFiles/utilization.txt $folder/utilization_sa.txt

# visualize the result
python Visualize_average_speed.py --path "$folder/time_task_number.txt" --minTaskNumber 3 --maxTaskNumber 8
