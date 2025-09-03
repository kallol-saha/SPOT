import os

import numpy as np

benchmark_folder = "benchmarks/FINAL_with_plans/without_obj_suggester/1p2b1c"

runs = [1, 3, 5, 7]
for r in runs:
    run_folder = f"run_{r}"

    run_folder_path = os.path.join(benchmark_folder, run_folder)
    node_info_path = os.path.join(run_folder_path, "node_info")

    # Number of files in node folder:
    num_nodes = len(os.listdir(node_info_path))

    goal_nodes = np.array([], dtype=np.int32)

    for i in range(1, num_nodes + 1):
        node_path = os.path.join(node_info_path, f"node_{i}.npz")
        node = np.load(node_path, allow_pickle=True)
        if node["is_goal"]:
            goal_nodes = np.append(goal_nodes, i)

    print(f"Goal nodes: {goal_nodes}")
    np.save(os.path.join(run_folder_path, "goal_nodes.npy"), goal_nodes)

# print(f"Number of nodes: {num_nodes}")

# Run 1 first node is goal
# Run 2 highest probability is goal
