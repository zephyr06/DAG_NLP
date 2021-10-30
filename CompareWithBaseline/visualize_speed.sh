#!/usr/bin/bash

title="task_number"
MaxTaskNumber=10

# clear buffer file content
> ResultFiles/utilization.txt
time_file="ResultFiles/time_$title.txt"
> $time_file

cd ../release
make -j8
cd ../CompareWithBaseline
for taskNumber in {6..10}
do
	echo "$title iteration is: $taskNumber"
	
	cd ../release
	./tests/GenerateTaskSet --taskSetType 2 --totalUtilization 0.4 --taskSetNumber 1000 --NumberOfProcessor 2 --taskType 1 --N $taskNumber
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
cp ResultFiles/time_task_number.txt ResultFiles/time_task_number_result_copy.txt
# visualize the result
python Visualize_average_speed.py --baseline "RM" --minTaskNumber 3 --maxTaskNumber $MaxTaskNumber

python Visualize_average_speed_automotive.py --path "time_record.txt" --ylim 40  --withFit 1 --maxTaskNumber 8
python Visualize_average_speed_automotive.py --path "accept_rate_record.txt" --ylim 60  --withFit 0 --maxTaskNumber 8
python Visualize_average_speed.py #speed
python Visualize_average_speed_automotive.py --path "accept_rate_record2.txt" --ylim 60  --withFit 0 --maxTaskNumber 8 --ylim 100

