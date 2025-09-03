import os

import numpy as np

eval_dir = "benchmarks/three_cubes_main_0"
# eval_dir = "benchmarks/first_try"
# eval_dir = "benchmarks/second_try_max_nodes_100"
folder_list = [
    d for d in os.listdir(eval_dir) if os.path.isdir(os.path.join(eval_dir, d))
]

print()
print("=" * 15 + "Displaying metrics for " + eval_dir + "=" * 15)
# print("=" * 30)
print("\n\n")

success_rates = []
total_examples = []
successful_examples_list = []
failed_examples_list = []
avg_path_lengths = []
avg_gen_nodes_list = []
avg_exp_nodes_list = []
avg_suggestions_made_list = []


for folder in folder_list:
    print("-" * 15 + " " + folder + " " + "-" * 15)
    print()

    # Load Metrics:
    metrics_path = os.path.join(eval_dir, folder, "metrics.npz")
    metrics = np.load(metrics_path, allow_pickle=True)

    # Extract values
    data_id = metrics["data_id"]
    success = metrics["success"]
    path_length = metrics["path_length"]
    gen_node_count = metrics["gen_node_count"]
    exp_node_count = metrics["exp_node_count"]
    total_suggestions_made = metrics["total_suggestions_made"]
    planning_time = metrics["planning_time"]

    # Calculate output
    # print(success)
    success_rate = np.mean(success)
    num_examples = len(success)
    success_mask = success == 1

    successful_examples = data_id[success_mask]
    failed_examples = data_id[~success_mask]

    execution_success = np.load(os.path.join(eval_dir, folder, "execution_success.npy"))
    execution_success_mask = execution_success == 1

    execution_successes = data_id[execution_success_mask]
    execution_success_rate = np.mean(execution_success)

    # Plan found but not executed:
    execution_failure_mask = success_mask & (~execution_success_mask)
    execution_failures = data_id[execution_failure_mask]

    # For all plans found:
    avg_path_length = np.mean(path_length[success_mask])
    avg_gen_nodes = np.mean(gen_node_count[success_mask])
    avg_exp_nodes = np.mean(exp_node_count[success_mask])
    avg_suggestions_made = np.mean(total_suggestions_made[success_mask])

    # For plans executed:
    avg_path_length_execution = np.mean(path_length[execution_success_mask])
    avg_gen_nodes_execution = np.mean(gen_node_count[execution_success_mask])
    avg_exp_nodes_execution = np.mean(exp_node_count[execution_success_mask])
    avg_suggestions_made_execution = np.mean(
        total_suggestions_made[execution_success_mask]
    )

    # Display everything
    print("Execution Success Rate = ", execution_success_rate)
    print("Planning Success Rate = ", success_rate)
    print()
    print("No. of Successfully Executed Examples = ", len(execution_successes))
    print("No. of Plans that failed to execute = ", len(execution_failures))
    print("No. of Plans not found = ", len(failed_examples))
    print()
    print("No. of Successful Plans Found = ", len(successful_examples))
    print("No. of Examples = ", num_examples)
    print()
    print("Executed = ", np.sort(execution_successes))
    print("Plan found but not executed = ", np.sort(execution_failures))
    print("Plans found = ", np.sort(successful_examples))
    print("No plans found = ", np.sort(failed_examples))
    print()

    print("*" * 10)

    print("Average Path Length of plans found = ", avg_path_length)
    print("Average Nodes Generated in plans found = ", avg_gen_nodes)
    print("Average Nodes Expanded in plans found = ", avg_exp_nodes)
    print("Average Suggestions Made in plans found = ", avg_suggestions_made)
    print()
    print("Average Path Length of executed plans = ", avg_path_length_execution)
    print("Average Nodes Generated in executed plans = ", avg_gen_nodes_execution)
    print("Average Nodes Expanded in executed plans = ", avg_exp_nodes_execution)
    print(
        "Average Suggestions Made in executed plans = ", avg_suggestions_made_execution
    )
    print()

    success_rates.append(success_rate)
    total_examples.append(num_examples)
    successful_examples_list.append(len(successful_examples))
    failed_examples_list.append(len(successful_examples))
    avg_path_lengths.append(avg_path_length)
    avg_gen_nodes_list.append(avg_gen_nodes)
    avg_exp_nodes_list.append(avg_exp_nodes)
    avg_suggestions_made_list.append(avg_suggestions_made)

print()
print("\n\n")
print("=" * 15 + "TOTAL METRICS" + "=" * 15)
print()

# Display everything
print("Total Planning Success Rate = ", np.mean(success_rates))
print("Total No. of Successful Plans Found = ", np.sum(successful_examples_list))
print("Total No. of Failed Examples = ", np.sum(failed_examples_list))
print("Total No. of Examples = ", np.sum(total_examples))
print()
print("Average of all Path Lengths = ", np.mean(avg_path_length))
print("Average of all Nodes Generated = ", np.mean(avg_gen_nodes))
print("Average of all Nodes Expanded = ", np.mean(avg_exp_nodes))
print("Average of all Suggestions Made = ", np.mean(avg_suggestions_made))
print()
