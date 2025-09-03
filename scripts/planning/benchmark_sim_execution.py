"""
Execute the plans produced by `benchmark_sim.py`.
"""

import argparse
import os
import shutil

import hydra
import numpy as np
from tqdm import tqdm

from spot.heuristics import Heuristic
from spot.pybullet_env.scene_gen.generate_scene import ObjectNotMovedException, Scene
from spot.utils.pcd_utils import apply_transform


def execute_plan(
    cfg,
    plan,
    heuristic: Heuristic,
    env: Scene,
    plan_folder: str = None,
    plot: bool = False,
) -> bool:
    current_state = env.initial_state

    # Start recording:
    if env.record:
        env.recording_cam.start_recording(plan_folder)

    for obj_id, T in plan:
        pcd, pcd_seg, _ = env.get_observation()

        # Visualize the transform
        apply_transform(pcd, pcd_seg, obj_id, T, plot=plot)

        # Make the transform
        try:
            obj_name = [name for name, _id in env.objects.items() if _id == obj_id][
                0
            ]  # Find the object name
            next_state = env.move_object(
                current_state,
                obj_name,
                T,
                teleport=False,
                fail_on_not_moved=True,
            )[3]
        except ObjectNotMovedException:
            print("Failed to execute plan!")
            return False

        current_state = next_state

    # Check if goal reached
    pcd, pcd_seg, _ = env.get_observation()

    if env.record:
        env.recording_cam.stop_recording()

    return heuristic(cfg, pcd, pcd_seg) == 0


def main(cfg, seed):
    # Create simulator
    env = Scene(
        cfg,
        seed=seed,
        gui=cfg.gui,
        robot=True,
    )

    benchmark_folder = os.path.join("benchmarks", f"{cfg.benchmark_name}_{seed}")

    # Loop through the datasets in the benchmark folder by listing folders inside:
    eval_datasets = [
        d
        for d in os.listdir(benchmark_folder)
        if os.path.isdir(os.path.join(benchmark_folder, d))
    ]
    for dataset in eval_datasets:
        # metrics = defaultdict(list)

        dataset_path = os.path.join(benchmark_folder, dataset)

        planning_metrics = np.load(
            os.path.join(dataset_path, "metrics.npz"), allow_pickle=True
        )
        planning_success = planning_metrics[
            "success"
        ]  # 0s and 1s for plan not found or plan found
        execution_success = np.zeros_like(
            planning_success
        )  # Initialize array for execution success
        data_ids = planning_metrics[
            "data_id"
        ]  # Unique number to locate the plan folder

        success_mask = planning_success == 1
        successful_examples = data_ids[success_mask]

        print("Executing dataset: ", dataset)
        for i in tqdm(successful_examples):
            data_id = int(i)

            plan_folder = os.path.join(dataset_path, str(data_id))

            # Load and set start state
            initial_state = np.load(
                os.path.join(
                    cfg.eval_dataset_folder,
                    dataset,
                    str(data_id) + "_initial_state.npz",
                ),
                allow_pickle=True,
            )
            env.set_initial_state(initial_state["initial_state"].item())
            env.reset()

            if "output_plan.npz" not in list(os.listdir(plan_folder)):
                print(
                    "This example does not have a plan. Expecting the plan to be in 'output_plan.npz'"
                )
                return

            # Load plan
            plan_data = np.load(
                os.path.join(plan_folder, "output_plan.npz"), allow_pickle=True
            )
            plan = zip(plan_data["object_order"], plan_data["transforms"])

            # Load heuristic
            data = np.load(
                os.path.join(cfg.eval_dataset_folder, dataset, str(data_id) + ".npz"),
                allow_pickle=True,
            )
            heuristic = data["heuristic"].item()

            try:
                success = execute_plan(
                    cfg, plan, heuristic, env, plan_folder, plot=False
                )
            except:
                success = False

            idx = np.where(data_ids == data_id)[0]
            execution_success[idx] = float(success)

        # Add execution success to output plan:
        np.save(os.path.join(dataset_path, "execution_success.npy"), execution_success)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Evaluate the planning algorithm")
    parser.add_argument(
        "--config",
        type=str,
        default="blocks.yaml",
        help="config for the planner",
    )
    args = parser.parse_args()

    with hydra.initialize(version_base=None, config_path="../../configs/blocks"):
        cfg = hydra.compose(args.config)

    for seed in range(5):
        print(f"--- seed {seed} ---")

        os.makedirs(os.path.join("benchmarks", f"{cfg.benchmark_name}_{seed}"), exist_ok=True)

        # Copy the config file to the benchmark folder

        shutil.copy(
            os.path.join("configs/blocks", str(args.config)),
            os.path.join(
                "benchmarks",
                f"{cfg.benchmark_name}_{seed}",
                "benchmark_sim_execution.yaml",
            ),
        )
        main(cfg, seed)
