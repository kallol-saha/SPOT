# Redo grasp and execution after moving each object

#!/bin/bash

# OBJECT_NAMES="rubber duck.\ blue box.\ wooden bowl"

if [ $# -lt 1 ]; then
    echo "Usage: $0 <config_name>"     # you need the config file name as argument, this should be inside the configs/ folder
    exit 1
fi


config_path=$(realpath "$1")         # Get the full path of the config file, so I can pass into any script anywhere
echo "Using config at $config_path"

# echo "Experiment to run now: $1"

source ~/miniconda3/etc/profile.d/conda.sh

# Starting in franka-panda-execution directory

# Define the file you want to check
# file="completed.txt"

# counter=1

# Connect to the robot:
# echo "Starting the control PC..."
# cd ~/franka-panda-execution/frankapy
# bash ./bash_scripts/start_control_pc.sh -i iam-dopey

cd ~/3DVTAMP
echo "Getting the initial keyframe..."

# Save keyframe
# cd ~/franka-panda-execution
conda activate ros_noetic
python get_keyframes_rgbd.py --config $config_path    # Get the initial keyframe
conda deactivate

echo "Keyframe saved."
# Launch video recording in new terminal
# gnome-terminal -- bash -c "bash record_video.sh \"$config_path\"; exec bash" &
# Run video collection in a different terminal
# gnome-terminal -- bash -c "./record_video.sh --config \"$config_path\"; exec bash"

# ------ Planning ------ #

echo "Getting initial point cloud, visualization should pop up..."

# First get the initial pcd:
set +e
cd ~/video_to_transforms
conda activate vid2trans #vid2transclone
python get_initial_pcd_real.py --config $config_path    # This saves real_initial_pcd.npz
conda deactivate
cp real_initial_pcd.npz ~/3DVTAMP/real_initial_pcd.npz

echo "Initial point cloud saved."
echo "Generating the plan..."
# Works till here!

# Generate the plan:
cd ~/3DVTAMP
conda activate spot
python plan_real_world.py --config $config_path # --overwrite      # This generates output_plan.npz inside the main folder
conda deactivate

echo "Plan generated, make sure you have checked the plan before proceeding."

# Move the plan to franka-panda-execution for execution
cp output_plan.npz ~/franka-panda-execution/output_plan.npz
cp output_plan.npz ~/video_to_transforms/output_plan.npz
cp plan_folder.npy ~/video_to_transforms/plan_folder.npy

# Get the object order
echo "Extracting object ids from the plan..."

conda activate spot
python extract_obj_ids.py --config $config_path    # Extract sequence of object ids from the saved plan and save it in obj_id_seq.txt, this is needed for grasp generation, see below
conda deactivate

echo "Object ids extracted."

# ---------------------- #

# Why do we need this loop? Because we need the object ids to be passed as arguments to contact_graspnet, without having to read an npz file
cd ~/3DVTAMP
declare -a numbers              # Declare an array to store the numbers
while IFS= read -r line; do     # Read the file line by line
    numbers+=("$line")          # Append the number on the current line to the array
done < obj_id_seq.txt           # Specify the input file

# Resetting the robot
# conda activate ros_noetic
# source frankapy/catkin_ws/devel/setup.bash
# python go_home.py
# conda deactivate

# echo "Robot has been reset"

echo "Numbers in the array:"
for i in "${!numbers[@]}"; do
# for ((i=2; i<=3; i++)); do
    num="${numbers[i]}"  # Get the current element
    # echo "Index: $i, Number: $num"
    # output_plan.npz is loaded in by execute_contact_graspnet_grasps.py inside execute_robot.sh and used to execute the plan
    # This function needs the config and the transition number
    cd ~/franka-panda-execution
    bash execute_transform.sh $i $num $config_path       # TODO: Add argument for objects prompt. Also make sure we can work around when gsam cannot see an object.

    # Save MDE data:
    echo "Saving MDE data"
    cd ~/video_to_transforms
    conda activate vid2trans
    python save_keyframe_pcd.py --config $config_path --transition_num $num
    conda deactivate
    echo "MDE data saved"

done

# Remove the object id sequence file
rm -rf obj_id_seq.txt

# Resetting the robot
# cd ~/franka-panda-execution
# conda activate ros_noetic
# source frankapy/catkin_ws/devel/setup.bash
# python go_home.py
# conda deactivate
