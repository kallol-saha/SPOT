import os

import numpy as np

from spot.utils.pcd_utils import (
    remove_outliers_from_full_pcd_table_bussing,
    transform_pcd,
)

extrinsics_file = "assets/extrinsics.npz"
# object_names = "yellow cup. green plate. pink plate. blue bowl"
object_names = "white cup. yellow bowl. red bowl. blue plate"

plate_thresh = 0.09
bowl_thresh = 0.05

extrinsics = np.load(extrinsics_file)
T_cam_to_world = extrinsics["T"]

object_list = [item.strip() for item in object_names.split(".") if item.strip()]
plates = []
bowls = []
cups = []
objects = []
for i, obj in enumerate(object_list):
    if "plate" in obj:
        objects.append("plate")
        plates.append(i)
    elif "bowl" in obj:
        objects.append("bowl")
        bowls.append(i)
    elif "cup" in obj:
        objects.append("cup")
        cups.append(i)


def check_feasibility(
    moved_object: int,
    pcd: np.ndarray,
    pcd_seg: np.ndarray,
    parent_pcd: np.ndarray,
    parent_pcd_seg: np.ndarray,
):
    feasible = True

    obj_pcd = pcd[pcd_seg == moved_object]
    obj_mean = (np.max(obj_pcd, axis=0) + np.min(obj_pcd, axis=0)) / 2

    obj_pcd_parent = parent_pcd[parent_pcd_seg == moved_object]
    obj_mean_parent = (
        np.max(obj_pcd_parent, axis=0) + np.min(obj_pcd_parent, axis=0)
    ) / 2

    if objects[moved_object] == "plate":
        # An already stacked plate cannot be moved
        other_plates = [x for x in plates if x != moved_object]

        for plate in other_plates:
            plate_pcd_parent = parent_pcd[parent_pcd_seg == plate]
            plate_mean_parent = (
                np.max(plate_pcd_parent, axis=0) + np.min(plate_pcd_parent, axis=0)
            ) / 2

            if (
                np.linalg.norm(plate_mean_parent[:2] - obj_mean_parent[:2])
                < plate_thresh
            ):
                feasible = False

        # Illegal move if a bowl or cup is already where it was moved

        for bowl in bowls:
            bowl_pcd = pcd[pcd_seg == bowl]
            bowl_mean = (np.max(bowl_pcd, axis=0) + np.min(bowl_pcd, axis=0)) / 2

            bowl_pcd_parent = parent_pcd[parent_pcd_seg == bowl]
            bowl_mean_parent = (
                np.max(bowl_pcd_parent, axis=0) + np.min(bowl_pcd_parent, axis=0)
            ) / 2

            if np.linalg.norm(bowl_mean[:2] - obj_mean[:2]) < plate_thresh:
                feasible = False

            if (
                np.linalg.norm(bowl_mean_parent[:2] - obj_mean_parent[:2])
                < plate_thresh
            ):
                feasible = False

        for cup in cups:
            cup_pcd = pcd[pcd_seg == cup]
            cup_mean = (np.max(cup_pcd, axis=0) + np.min(cup_pcd, axis=0)) / 2

            cup_pcd_parent = parent_pcd[parent_pcd_seg == cup]
            cup_mean_parent = (
                np.max(cup_pcd_parent, axis=0) + np.min(cup_pcd_parent, axis=0)
            ) / 2

            if np.linalg.norm(cup_mean[:2] - obj_mean[:2]) < plate_thresh:
                feasible = False

            if np.linalg.norm(cup_mean_parent[:2] - obj_mean_parent[:2]) < plate_thresh:
                feasible = False

    elif objects[moved_object] == "bowl":
        # An already stacked bowl cannot be moved
        other_bowls = [x for x in bowls if x != moved_object]

        for bowl in other_bowls:
            bowl_pcd_parent = parent_pcd[parent_pcd_seg == bowl]
            bowl_mean_parent = (
                np.max(bowl_pcd_parent, axis=0) + np.min(bowl_pcd_parent, axis=0)
            ) / 2

            if np.linalg.norm(bowl_mean_parent[:2] - obj_mean_parent[:2]) < bowl_thresh:
                feasible = False

        # Illegal move if a cup is already where it was moved

        for cup in cups:
            cup_pcd = pcd[pcd_seg == cup]
            cup_mean = (np.max(cup_pcd, axis=0) + np.min(cup_pcd, axis=0)) / 2

            cup_pcd_parent = parent_pcd[parent_pcd_seg == cup]
            cup_mean_parent = (
                np.max(cup_pcd_parent, axis=0) + np.min(cup_pcd_parent, axis=0)
            ) / 2

            if np.linalg.norm(cup_mean[:2] - obj_mean[:2]) < plate_thresh:
                feasible = False

            if np.linalg.norm(cup_mean_parent[:2] - obj_mean_parent[:2]) < plate_thresh:
                feasible = False

    return feasible


def score_goal(goal_node: int, node_folder: str):
    node_id = goal_node
    prev_feasible = True
    parent_id = -1

    collision_sum = 0
    deviation_sum = 0
    probability_sum = 0

    while parent_id != 0:
        node_path = os.path.join(node_folder, f"node_{node_id}.npz")
        node = np.load(node_path, allow_pickle=True)

        pcd = node["pcd"]
        pcd_seg = node["pcd_seg"]
        moved_object = node["moved_object"]
        parent_id = node["parent"]

        if parent_id != 0:
            parent_node_path = os.path.join(node_folder, f"node_{parent_id}.npz")
            parent_node = np.load(parent_node_path, allow_pickle=True)
            parent_pcd = parent_node["pcd"]
            parent_pcd_seg = parent_node["pcd_seg"]
        else:
            parent_node_path = os.path.join(run_folder_path, "initial_pcd.npz")
            parent_node = np.load(parent_node_path, allow_pickle=True)
            parent_pcd = parent_node["initial_pcd"]
            parent_pcd_seg = parent_node["initial_pcd_seg"]
            parent_pcd, parent_pcd_seg = remove_outliers_from_full_pcd_table_bussing(
                np.arange(4), parent_pcd, parent_pcd_seg
            )

        pcd = transform_pcd(pcd, T_cam_to_world)

        feasible = check_feasibility(
            moved_object, pcd, pcd_seg, parent_pcd, parent_pcd_seg
        )

        feasible = feasible and prev_feasible
        # if not feasible:
        #     break

        collision_sum += node["collision"]
        deviation_sum += node["deviation"]
        probability_sum += node["probability"]

        prev_feasible = feasible
        node_id = parent_id
        if parent_id == 0:
            break

    return feasible, [collision_sum, deviation_sum, probability_sum]


benchmark_folder = "benchmarks/FINAL_with_plans/without_obj_suggester/1p2b1c"

runs = [5, 7]
for r in runs:
    run_folder = f"run_{r}"

    run_folder_path = os.path.join(benchmark_folder, run_folder)
    node_info_path = os.path.join(run_folder_path, "node_info")

    # Number of files in node folder:
    num_nodes = len(os.listdir(node_info_path))

    goal_nodes = np.load(os.path.join(run_folder_path, "goal_nodes.npy"))
    collisions = []
    probabilities = []

    for g in goal_nodes:
        score, sums = score_goal(
            g, node_info_path
        )  # The score is hand-specified to debug if the goal heuristic is working
        collisions.append(sums[0])
        probabilities.append(sums[2])
        if score:
            print(
                f"Node {g} is feasible, collision: {sums[0]}, deviation: {sums[1]}, probability: {sums[2]}"
            )
            # np.save(os.path.join(run_folder_path, f"goal.npy"), g)
        else:
            print(
                f"Node {g} is NOT feasible, collision: {sums[0]}, deviation: {sums[1]}, probability: {sums[2]}"
            )

    collisions = np.array(collisions)
    probabilities = np.array(probabilities)

    collisions[probabilities < 1.0] = 1000

    print("---------------------------------")

    # Main goal check condition:
    best_goal = goal_nodes[np.argmin(collisions)]
    print(
        f"Best goal: {best_goal}, collision: {collisions[np.argmin(collisions)]}, probability: {probabilities[np.argmin(collisions)]}"
    )
    np.save(os.path.join(run_folder_path, f"goal.npy"), best_goal)

    print("---------------------------------")


# print(f"Number of nodes: {num_nodes}")

# 2p1b1c: 4
