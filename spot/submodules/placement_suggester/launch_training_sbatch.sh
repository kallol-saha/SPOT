#!/bin/bash
#SBATCH --nodes=1
#SBATCH --partition=GPU  # Replace 'your_partition' with the appropriate partition name
#SBATCH --ntasks-per-node=1
#SBATCH --time=96:00:00     # Replace 'your_time_limit' with the desired time limit
#SBATCH --gres=gpu:1              # If you need a GPU, adjust this line accordingly
#SBATCH --mem=64G         # Replace 'your_memory' with the memory requirement
#SBATCH --nodelist=compute-1-9    # Specify the desired compute node
#SBATCH -o /home/jennyw2/sbatch_logs/output/job_%A_%a.log  # %j will be replaced with the job ID
#SBATCH -e /home/jennyw2/sbatch_logs/error/job_%A_%a.err  # %j will be replaced with the job ID
#SBATCH --mail-type=END,FAIL
#SBATCH --mail-user=jennyw2@andrew.cmu.edu  # Replace with your email address
#SBATCH --array=1-5  # Adjust the range to match the number of tasks







set -x
set -u
set -e
module load singularity

# Define the Singularity image file
SINGULARITY_IMAGE=/home/jennyw2/singularity/taxposed.sif

# Command to run inside the Singularity container
#COMMANDS=(
#	"python scripts/train_residual_flow_multimodal.py gradient_clipping=1e-1"
#	"python scripts/train_residual_flow_multimodal.py gradient_clipping=1e-2"
#	"python scripts/train_residual_flow_multimodal.py gradient_clipping=1e-3"
#	"python scripts/train_residual_flow_multimodal.py gradient_clipping=1e-5"
#	"python scripts/train_residual_flow_multimodal.py gradient_clipping=1e0"
#	"python scripts/train_residual_flow_multimodal.py gradient_clipping=1e1"
#	"python scripts/train_residual_flow_multimodal.py gradient_clipping=1e2"	
#)

COMMANDS=(
	"python scripts/train_residual_flow_multimodal.py rot_sample_method=axis_angle gradient_clipping=1e-1"
	"python scripts/train_residual_flow_multimodal.py rot_sample_method=axis_angle gradient_clipping=1e-2"
	"python scripts/train_residual_flow_multimodal.py rot_sample_method=axis_angle gradient_clipping=1e-3"
	"python scripts/train_residual_flow_multimodal.py rot_sample_method=axis_angle gradient_clipping=1e-4"
	"python scripts/train_residual_flow_multimodal.py rot_sample_method=axis_angle gradient_clipping=1e-5"
)

# Run the Singularity container
time \
	singularity exec --nv \
	-B /home/jennyw2/code/taxpose/data/mug_place/:/code/placement_suggester/data \
	-B /home/jennyw2/code/placement_suggester/logs:/code/placement_suggester/logs \
	$SINGULARITY_IMAGE \
	bash -c "source activate taxposed && \
		${COMMANDS[$SLURM_ARRAY_TASK_ID - 1]} \
		dataset_root=/code/placement_suggester/data/train_data/renders \
		test_dataset_root=/code/placement_suggester/data/test_data/renders \
		log_dir=/code/placement_suggester/logs"

#               CUDA_VISIBLE_DEVICES=0 \

# singularity exec \
# 	-B /home/$(whoami)/code/placement_suggester:/opt/$(whoami)/code \
# 	-B /scratch/$(whoami)/logs:/opt/logs \
# 	$SINGULARITY_IMAGE $COMMAND_TO_RUN
# 	# -B /scratch/$(whoami)/data:/data \
# 	# -B /scratch/$(whoami)/artifacts:/opt/artifacts \
# 	# --bind /home/jenny/code/placement_suggester/data:/home/jenny/code/placement_suggester/data \
	
