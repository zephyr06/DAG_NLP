#!/bin/bash

# ***************************************************
# ************** Adjust settings there **************
title="RTDA"
MinTaskNumber=3
MaxTaskNumber=30
TaskNumberArray=(3 4 5 6 7 8 9 10 15 20 25 30)
# TaskNumberArray=(6 7 8 9 10)
## no separator '/' at the end of the path
# ROOT_PATH="/home/zephyr/Programming/DAG_NLP" 
ROOT_PATH="/home/dong/workspace/DAG_NLP"
# ROOT_PATH="/home/zephyr/Programming/batch_test_DAG_NLP/VerucchiOnly/N3_10" # for final batch test
RESULTS_PATH="$ROOT_PATH/CompareWithBaseline/BatchResultsForRTAS2023/$title"
# ***************************************************
# ***************************************************

./extract_txt_for_RTDA_batch.sh
echo "Plot from results in directory: $RESULTS_PATH"

python $ROOT_PATH/CompareWithBaseline/BatchResultsForRTAS2023/$title/VisualizeRtdaPerformanceRTAS2023.py --minTaskNumber $MinTaskNumber \
    --title $title --maxTaskNumber $MaxTaskNumber --result_file_path $RESULTS_PATH \
    --taskNumberList ${TaskNumberArray[@]}

cp $ROOT_PATH/CompareWithBaseline/BatchResultsForRTAS2023/$title/*.pdf $ROOT_PATH/CompareWithBaseline/BatchResultsForRTAS2023/$title/scripts_and_figures_backup/
cp $ROOT_PATH/CompareWithBaseline/BatchResultsForRTAS2023/$title/*.sh $ROOT_PATH/CompareWithBaseline/BatchResultsForRTAS2023/$title/scripts_and_figures_backup/
cp $ROOT_PATH/CompareWithBaseline/BatchResultsForRTAS2023/$title/*.py $ROOT_PATH/CompareWithBaseline/BatchResultsForRTAS2023/$title/scripts_and_figures_backup/

