import argparse
import os
import re
import shutil
import time

import hydra
import numpy as np
from omegaconf import OmegaConf
from tqdm import tqdm

from spot.evaluation import update_plan_metrics
from spot.mde import MDE
from spot.object_suggester import ObjectSuggester
from spot.planners.a_star import Astar as Planner
from spot.suggester import Suggester
from spot.suggester_utils import set_seed


def main(cfg, seed, overwrite: bool = False):
    # Reproducibility:
    set_seed(
        seed
    )  # NOTE that the torch seed gets reset in the get_data() function inside TestPointCloudDataset, should be fine for now.
    OmegaConf.update(cfg, "seed", seed, force_add=True)

    # Prepare the output directory for the benchmark:
    output_dir = os.path.join("benchmarks", f"{cfg.benchmark_name}_{seed}")
    if os.path.exists(output_dir):
        if not overwrite:
            raise FileExistsError(
                f"The directory {output_dir} already exists and cannot be "
                "overwritten. Use --overwrite if you want to overwrite"
            )
        shutil.rmtree(output_dir)
    os.makedirs(output_dir, exist_ok=True)

    # Save the configs that are used into the output folder
    shutil.copy(
        os.path.join("configs/blocks", str(args.config)),
        os.path.join(output_dir, "plan.yaml"),
    )
    shutil.copy(
        os.path.join("configs/training", str(cfg.suggester.config_file)),
        os.path.join(output_dir, "suggester.yaml"),
    )

    # Load models and planner
    suggester = Suggester(cfg)
    mde = MDE(cfg.mde)
    object_suggester = ObjectSuggester(cfg.object_suggester)
    planner = Planner(cfg, suggester, mde, object_suggester)

    # Inside each benchmark, there is a directory for each eval folder
    # inside each eval folder, there is a directory for each example

    for folder in cfg.eval_folders:
        eval_data_dir = os.path.join(cfg.eval_folder_location, folder)

        if os.path.exists(eval_data_dir):
            # Prepare the eval output directory
            eval_output_dir = os.path.join(output_dir, folder)
            os.makedirs(eval_output_dir, exist_ok=True)

            # input_data = [f for f in os.listdir(eval_data_dir) if f.endswith(".npz")]
            pattern = re.compile(r"^\d+\.npz$")  # To match all npz files
            input_data = [f for f in os.listdir(eval_data_dir) if pattern.match(f)]

            metrics = {}

            print("Evaluating dataset " + folder)

            for data in tqdm(input_data):
                data_id = int(data[:-4])

                pcd_data = np.load(os.path.join(eval_data_dir, data), allow_pickle=True)
                initial_pcd = pcd_data["initial_pcd"]
                initial_pcd_seg = pcd_data["initial_pcd_seg"]
                heuristic = pcd_data["heuristic"].item()

                # While loop is for random rollouts ablation only
                plan = None
                total_exp_node_count = 0
                while (
                    plan is None
                    and total_exp_node_count < cfg.planner.max_expanded_nodes
                ):
                    plan_start = time.time()
                    plan, start_node, pruned_node_count, exp_node_count = planner.plan(
                        initial_pcd,
                        initial_pcd_seg,
                        heuristic,
                    )
                    planning_time = time.time() - plan_start

                    # Update benchmarked metrics from the recent plan
                    metrics = update_plan_metrics(
                        cfg,
                        data_id,
                        eval_output_dir,
                        plan,
                        start_node,
                        initial_pcd,
                        initial_pcd_seg,
                        planning_time,
                        pruned_node_count,
                        metrics,
                        remove_outliers=False,
                    )

                    # Save metrics after each data point to preserve progress
                    np.savez(os.path.join(eval_output_dir, "metrics.npz"), **metrics)

                    if cfg.ablation != "random_rollouts":
                        break  # Done after one planning call

                    total_exp_node_count += exp_node_count
                    print("Total nodes expanded so far:", total_exp_node_count)

        else:
            raise FileExistsError(
                "The eval folder "
                + folder
                + " does not exist inside "
                + cfg.eval_folder_location
            )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Evaluate the planning algorithm")
    parser.add_argument(
        "--config",
        type=str,
        default="blocks.yaml",
        help="config for the planner",
    )
    parser.add_argument(
        "--overwrite",
        default=False,
        action="store_true",
        help="force overwrite existing results",
    )
    args = parser.parse_args()

    with hydra.initialize(version_base=None, config_path="../../configs/blocks"):
        cfg = hydra.compose(args.config)

    for seed in range(5):
        print(f"--- seed {seed} ---")
        main(cfg, seed, overwrite=args.overwrite)
