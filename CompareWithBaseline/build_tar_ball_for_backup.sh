#!/bin/bash

ROOT_PATH="/home/dong/workspace/DAG_NLP"
# ROOT_PATH="/home/zephyr/Programming/batch_test_DAG_NLP/VerucchiOnly/N3_10"
backup_dir="backup"
# ***************************************************

current_backup_files=()
for DIR in `find $ROOT_PATH -type d -name $backup_dir` ; do
  # echo $DIR
  cd $DIR
  files=`ls`
  for file in $files ; do
    if [[ ${file:(-7):7} != ".tar.gz" ]] ; then
      if [[ ! ${files[*]} =~ $file.tar.gz ]]; then
        echo "Make tar ball of directory: $DIR/$file"
        tar -czf $file.tar.gz $file
        current_backup_files+=("$DIR/$file.tar.gz")
      fi
    else
      current_backup_files+=("$DIR/$file")
    fi
  done
done

echo "Summary of current backup tar balls:"
for file in ${current_backup_files[@]} ; do
  echo $file
done