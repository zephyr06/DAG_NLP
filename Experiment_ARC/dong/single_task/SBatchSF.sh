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
export MODULEPATH="/projects/rtss_let/modules/tinkercliffs-rome/all:$MODULEPATH"
module --ignore_cache spider GTSAM
module reset
module load GTSAM/4.1.1-foss-2021b

## No more needed after success compilation ##
# cmake -DCMAKE_BUILD_TYPE=RELEASE ..
# make BatchSF_TOM

N=$1
file_index=$2
end_index=$3
./scripts/BatchSF_TOM --N $N --begin $file_index --end $end_index
