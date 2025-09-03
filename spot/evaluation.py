"""
The idea:

evaluate_plan() should do everything needed for saving metrics and visualizing the plan. Remember to also save the graph somehow.

evaluate_execution() should do everything needed for saving execution metrics
"""

import os

import numpy as np
from spot.visualization.graph import create_graph

# Global flag to control graph visualization across the project
VISUALIZE_GRAPH = False


def count_nodes_recursively(node, gen_node_count=0, exp_node_count=0):
    gen_node_count += 1
    if node.expanded:
        exp_node_count += 1
    for child in node.children:
        gen_node_count, exp_node_count = count_nodes_recursively(
            child, gen_node_count, exp_node_count
        )

    return gen_node_count, exp_node_count


def evaluate_plan(
    cfg,
    plan_output_dir,
    plan,
    start_node,
    initial_pcd,
    initial_pcd_seg,
    remove_outliers: bool = True,
):
    if plan is not None:
        found_plan = True
        np.savez(
            os.path.join(plan_output_dir, "output_plan.npz"), **plan
        )  # Save the plan
        path_length = len(plan["object_order"])

    else:
        found_plan = False
        path_length = -1

    gen_node_count, exp_node_count = count_nodes_recursively(start_node)
    total_suggestions_made = (
        exp_node_count * cfg.planner.num_objects * cfg.suggester.num_suggestions
    )

    if VISUALIZE_GRAPH:
        create_graph(
            cfg,
            start_node,
            initial_pcd,
            initial_pcd_seg,
            plan_output_dir,
            remove_outliers=remove_outliers,
        )

    return (
        found_plan,
        path_length,
        gen_node_count,
        exp_node_count,
        total_suggestions_made,
    )
    # Evaluated edges TODO: Is this only for the MDE?


def update_plan_metrics(
    cfg,
    data_id,
    eval_output_dir,
    plan,
    start_node,
    initial_pcd,
    initial_pcd_seg,
    planning_time,
    pruned_node_count,
    metrics={},
    remove_outliers: bool = True,
):
    # Prepare output directory for plan
    plan_output_dir = os.path.join(eval_output_dir, str(data_id))
    os.makedirs(plan_output_dir, exist_ok=True)

    # Initialize metrics
    if not metrics:
        metrics["data_id"] = np.array([])
        metrics["success"] = np.array([])
        metrics["path_length"] = np.array([])
        metrics["gen_node_count"] = np.array([])
        metrics["exp_node_count"] = np.array([])
        metrics["pruned_node_count"] = np.array([])
        metrics["total_suggestions_made"] = np.array([])
        metrics["planning_time"] = np.array([])

    (
        found_plan,
        path_length,
        gen_node_count,
        exp_node_count,
        total_suggestions_made,
    ) = evaluate_plan(
        cfg,
        plan_output_dir,
        plan,
        start_node,
        initial_pcd,
        initial_pcd_seg,
        remove_outliers=remove_outliers,
    )

    metrics["data_id"] = np.append(metrics["data_id"], data_id)
    metrics["success"] = np.append(metrics["success"], found_plan)
    metrics["path_length"] = np.append(metrics["path_length"], path_length)
    metrics["gen_node_count"] = np.append(metrics["gen_node_count"], gen_node_count)
    metrics["exp_node_count"] = np.append(metrics["exp_node_count"], exp_node_count)
    metrics["pruned_node_count"] = np.append(
        metrics["pruned_node_count"], pruned_node_count
    )
    metrics["total_suggestions_made"] = np.append(
        metrics["total_suggestions_made"], total_suggestions_made
    )
    metrics["planning_time"] = np.append(metrics["planning_time"], planning_time)

    return metrics


def evaluate_execution():
    pass
