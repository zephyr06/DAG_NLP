#!/bin/bash

ROOT_PATH="/home/dong/workspace/DAG_NLP"
backup_dir="backup"
# ***************************************************

current_backup_files=()
for DIR in `find $ROOT_PATH -type d -name $backup_dir` ; do
  # echo $DIR
  cd $DIR
  files=`ls`
  for file in $files ; do
    if [[ ${file:(-4):4} != ".tar" ]] ; then
      if [[ ! ${files[*]} =~ $file.tar ]]; then
        echo "Make tar ball of directory: $DIR/$file"
        tar -cf $file.tar $file
        current_backup_files+=("$DIR/$file.tar")
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