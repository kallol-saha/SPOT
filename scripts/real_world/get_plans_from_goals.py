import os

import numpy as np
from tqdm import tqdm

benchmark_folder = "benchmarks/FINAL/ours2/1p2b1c"
# benchmark_folder = "benchmarks/FINAL_with_plans/random/1p2b1c"

runs = [4]
for r in tqdm(runs):
    run_folder = f"run_{r}"

    run_folder_path = os.path.join(benchmark_folder, run_folder)
    node_info_path = os.path.join(run_folder_path, "node_info")

    goal_node_id = np.load(os.path.join(run_folder_path, "goal.npy"))

    transforms = []
    moved_objects = []

    node_id = goal_node_id

    while True:
        node_path = os.path.join(node_info_path, f"node_{node_id}.npz")
        node = np.load(node_path, allow_pickle=True)

        transform = node["T"][None, :]
        moved_object = node["moved_object"]

        transforms.append(transform)
        moved_objects.append(moved_object)

        parent_id = node["parent"]
        if parent_id == 0:
            break

        node_id = parent_id

    # Reverse the lists:
    transforms = transforms[::-1]
    moved_objects = moved_objects[::-1]

    # Convert to numpy arrays:
    transforms = np.concatenate(transforms, axis=0)
    moved_objects = np.array(moved_objects)

    np.savez(
        os.path.join(run_folder_path, "output_plan.npz"),
        transforms=transforms,
        object_order=moved_objects,
    )
