#!/usr/bin/bash
COUNT_PATH="/home/dong/workspace/DAG_NLP/TaskData"
# COUNT_PATH=/home/zephyr/Programming/DAG_NLP/TaskData
# COUNT_PATH="/home/dong/workspace/DAG_NLP/release/generatedNewTaskset"
COUNT_PATH="/projects/rtss_let/dong/DAG_NLP/TaskData"

echo current results count in dir: $COUNT_PATH

for dir in `ls $COUNT_PATH`; do
    if [[ $dir =~ N.* ]]; then
        echo $dir: `ls $COUNT_PATH/$dir | wc -l`
    fi
done
