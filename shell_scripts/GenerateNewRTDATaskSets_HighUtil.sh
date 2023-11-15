#!/usr/bin/bash

#@@@ parallelFactor is set in parameters.yaml @@@#

# ************** Adjust settings there **************
N=(5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21)
numPerThread=(200 200 200 200 200 200 200 200 200 200 200 200 200 200 200 200 200)
# N=(5 7 9 11 13 15 17 19 21)
# numPerThread=(200 200 200 200 200 200 200 200 200)
taskSetNumber=1000
numberOfProcessor=4
minUtilizationPerCore=0.9
maxUtilizationPerCore=0.9
SF_ForkNum=-1 # negative means random number of forks
fork_sensor_num_min=2
fork_sensor_num_max=9
numCauseEffectChain=-1 # negative means random number of chains
outDir="generatedNewTaskset"

# ***************************************************
cd ../release
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . --config Release --target GenerateTaskSet -j4

rm $outDir -rf
mkdir $outDir
cd $outDir
echo "New TaskSet Configs:" > configs.log
echo "taskNumber_list: ${N[@]}" >> configs.log
echo "taskSetNumber: $taskSetNumber" >> configs.log
echo "numberOfProcessor: $numberOfProcessor" >> configs.log
echo "minUtilizationPerCore: $minUtilizationPerCore" >> configs.log
echo "maxUtilizationPerCore: $maxUtilizationPerCore" >> configs.log
echo "SF_ForkNum: $SF_ForkNum" >> configs.log
echo "fork_sensor_num_min: $fork_sensor_num_min" >> configs.log
echo "fork_sensor_num_max: $fork_sensor_num_max" >> configs.log
echo "numCauseEffectChain: $numCauseEffectChain" >> configs.log

for (( idx = 0 ; idx < ${#N[@]}; idx++ )); do
    taskNumber=${N[idx]}
    echo "Generating task sets for N$taskNumber"
    mkdir N$taskNumber
    for taskSetStartNumber in $(seq 0 ${numPerThread[idx]} $((taskSetNumber - 1))); do
        ../tests/GenerateTaskSet --taskSetNameStartIndex $taskSetStartNumber \
            --taskSetNumber $(($taskSetStartNumber + ${numPerThread[idx]})) \
            --N $taskNumber \
            --numberOfProcessor $numberOfProcessor \
            --minUtilizationPerCore $minUtilizationPerCore \
            --maxUtilizationPerCore $maxUtilizationPerCore \
            --outDir "release/$outDir/N$taskNumber/" \
            --clearOutputDir 0 \
            --SF_ForkNum $SF_ForkNum \
            --fork_sensor_num_min $fork_sensor_num_min \
            --fork_sensor_num_max $fork_sensor_num_max \
            --numCauseEffectChain $numCauseEffectChain \
            &
    done
    wait
done

echo "TaskSet Creation Time: $(date +"%Y%m%d")" | cat - configs.log > temp && mv temp configs.log

wait
exit 0