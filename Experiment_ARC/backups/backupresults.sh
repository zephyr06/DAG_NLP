ROOT_PATH=/projects/rtss_let/dong/DAG_NLP
# ROOT_PATH=/home/dong/workspace/DAG_NLP
# ROOT_PATH=/home/zephyr/Programming/DAG_NLP
if [[ -n "${TMPDIR}" ]]; then
TEMP_PATH=${TMPDIR}/dag
else
TEMP_PATH=$ROOT_PATH
fi

backup_file_name=ARC_Results_Backup_DAG_NLP_$(date +"%Y%m%d%H%M%S")
echo "backup in $ROOT_PATH/Experiment_ARC/backups/${backup_file_name}.tar.gz"

cd ${TEMP_PATH}/Experiment_ARC/backups
mkdir ${backup_file_name}

cp $ROOT_PATH/TaskData/configs.log ./${backup_file_name}/
cp -r $ROOT_PATH/TaskData/N* ./${backup_file_name}/

# cp -r $ROOT_PATH/Experiment_ARC/dong ./${backup_file_name}/slurm_outputs/
# cp $ROOT_PATH/Experiment_ARC/*.out ./${backup_file_name}/slurm_outputs/

tar -czf ${backup_file_name}.tar.gz ${backup_file_name}
rm ${backup_file_name} -r
mv ${backup_file_name}.tar.gz ${ROOT_PATH}/Experiment_ARC/backups/
