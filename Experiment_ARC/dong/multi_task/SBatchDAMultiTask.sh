#!/bin/bash
#SBATCH -J BatchRunAll
#SBATCH --account=ev_charging
#SBATCH --partition=normal_q
#SBATCH --nodes=1 --ntasks-per-node=1 --cpus-per-task=1
#SBATCH --time=0-06:00:00 # 6 hours

#SBATCH --mail-user=dongli@vt.edu
#SBATCH --mail-type=BEGIN  # send email when job begins
#SBATCH --mail-type=END    # send email when job ends
#SBATCH --mail-type=FAIL   # send email when job aborts

echo "Current job id is: $SLURM_JOB_ID"

## change directory and load modules ##
ROOT_PATH=/projects/rtss_let/dong/DAG_NLP
# ROOT_PATH=/home/dong/workspace/DAG_NLP
cd $ROOT_PATH/release
module reset
module load GTSAM/4.1.1-foss-2021b

## No more needed after success compilation ##
# cmake -DCMAKE_BUILD_TYPE=RELEASE ..
# make BatchDA_TOM

N=$1
MinFileIndex=$2
files_per_task=$3
MaxFileIndex=$4

## Launch multiple tasks in one node, make sure applied for adequate ntasks-per-node
for file_index in $(seq $MinFileIndex $files_per_task $MaxFileIndex); do
	end_index=$((file_index + files_per_task))
    echo "./scripts/BatchDA_TOM --N $N --begin $file_index --end $end_index &"
    ./scripts/BatchDA_TOM --N $N --begin $file_index --end $end_index &
done

## Wait for all of the background tasks to finish
wait
exit 0