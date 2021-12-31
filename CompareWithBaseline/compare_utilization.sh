#!/usr/bin/bash

title="utilization"

folder="compare_utilization"
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
for i in $(seq 0.1 0.1 0.9) 
do
	echo "$title iteration is: $i"
	
	cd ../release
	./tests/GenerateTaskSet --taskSetType 2 --totalUtilization $i --taskSetNumber 10 --NumberOfProcessor 2 --N 5 --taskType 1 --deadlineType 1
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
cp ResultFiles/utilization.txt $folder/util_core.txt

# visualize the result
cd $folder
python Visualize_util.py --baseline "RM"

# other parameters:

# barrierBase: 100
# coolingRateSA: 0.1
# debugMode: 0
# deltaInitialDogleg: 1e5
# weightDDL_factor: 1.0
# deltaOptimizer: 1e-1
# exactJacobian: 0
# initialLambda: 1e-4
# lowerLambda: 1e-15
# makespanWeight: 0
# noiseModelSigma: 1
# AcceptSchedulError: 1e1
# optimizerType: 2 # 1 means dogleg, 2 means LM
# overlapMode: 1
# parallelFactor: 0.2
# punishmentInBarrier: 1e2
# randomInitialize: 0
# readTaskMode: orig
# withAddedSensorFusionError: 1
# ElimnateLoop_Max: 50
# zeroJacobianDetectTol: 1e-7
# maxIterations: 1000
# moreElimination: 0
# weightPrior_factor: 0
# initializeMethod: 2
# timeScaleFactor: 1e2
# stepJacobianIteration: 1.5
# maxJacobianIteration: 30
# priorityMode: RM
# relativeErrorTolerance: 1e-5
# runMode: compare
# SA_iteration: 100000
# sensorFusionTolerance: 50
# FreshTol: 1000000000
# temperatureSA: 100000000
# testDataSetName: test_n5_v44
# tightEliminate: 0
# toleranceEliminator: 1
# upperLambda: 1e60
# weightLogBarrier: -1.0
# setUseFixedLambdaFactor: 1
# batchTestMethod: 1 # 1 means NLP, 2 means SA, 0 means initial
# numericalJaobian: 0
# TaskSetType: 1 # 1 means total random, 2 means automobile tasksets
