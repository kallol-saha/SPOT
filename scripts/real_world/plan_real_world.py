import argparse
import os
import shutil

import numpy as np
from omegaconf import OmegaConf

# from spot.heuristics import table_bussing_heuristic_quadratic_xyz as heuristic
# from spot.heuristics import table_bussing_two_plates_cup_and_bowl as heuristic
# from spot.heuristics import table_bussing_two_plates_cup_and_bowl_per_object as heuristic
from spot.heuristics import table_bussing_goal as goal
from spot.heuristics import table_bussing_heuristic as heuristic

# TODO: These imports should actually be hyperparameters
from spot.mde import MDE as MDE
from spot.object_suggester import ObjectSuggester

# from spot.mde import NoMDE as MDE
from spot.planners.a_star import Astar as Planner

# from spot.planners.greedy_rollout import GreedyRollout as Planner
# from spot.planners.random_rollout import RandomRollout as Planner
from spot.suggester import Suggester
from spot.suggester_utils import set_seed
from spot.visualization.graph import create_graph


# Global flag to control graph visualization across the project
VISUALIZE_GRAPH = False


def main(cfg, overwrite: bool = False):
    # Reproducibility:
    set_seed(cfg.seed)
    # NOTE that the torch seed gets reset in the get_data() function inside
    # TestPointCloudDataset, should be fine for now.

    # Save the absolute path to the plan folder:
    plan_folder_absolute = os.path.abspath(cfg.plan_folder)
    np.save("plan_folder.npy", plan_folder_absolute)

    if os.path.exists(os.path.join(cfg.plan_folder, "plan.yaml")) and not overwrite:
        # NOTE: We cannot prevent this for execution metrics, they need to be saved
        # inside the same directory, by loading in that directory's configs.
        # raise FileExistsError(
        #     f"The experiment {cfg.plan_folder} already exists and cannot be "
        #     "overwritten. Remove manually if you want to overwrite"
        # )
        _ = input(
            "A plan already exists in this folder. Press enter to execute this plan."
        )
        return None
    else:
        os.makedirs(cfg.plan_folder, exist_ok=True)

    # Save the config for each experiment into the experiments output folder
    shutil.copy(str(args.config), cfg.plan_folder + "/plan.yaml")
    shutil.copy(
        "configs/training/" + str(cfg.suggester.config_file),
        cfg.plan_folder + "/suggester.yaml",
    )

    # Load in the initial point cloud:
    example = np.load(cfg.real_world.input_pcd_path, allow_pickle=True)
    initial_pcd = example["initial_pcd"]
    initial_pcd_seg = example["initial_pcd_seg"]

    np.savez(os.path.join(cfg.plan_folder, "initial_pcd.npz"), **example)
    # np.save(output_dir + "/initial_pcd.npy", initial_pcd)
    # np.save(output_dir + "/initial_pcd_seg.npy", initial_pcd_seg)

    # plot_pcd(initial_pcd, initial_pcd_seg == 1)

    suggester = Suggester(cfg)
    mde = MDE(cfg.mde)
    object_suggester = ObjectSuggester(cfg.object_suggester)
    planner = Planner(cfg, suggester, mde, object_suggester)
    plan, start_node, _ = planner.plan(initial_pcd, initial_pcd_seg, heuristic, goal)

    if plan is None:
        print("No plan found!")
    else:
        print(plan)
        np.savez(os.path.join(cfg.plan_folder, "output_plan.npz"), **plan)
        # Also save the plan in this folder so it is easily accessed by the bash script:
        np.savez("output_plan.npz", **plan)

    # TODO: After getting the plan, we need to save all the visualizations and metrics from it. All of this maybe wrapped inside a single evaluation class
    if VISUALIZE_GRAPH:
        create_graph(cfg, start_node, initial_pcd, initial_pcd_seg, cfg.plan_folder)

    _ = input(
        "Check the plan before continuing!! Press enter if you want to execute this plan"
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Evaluate the planning algorithm")
    parser.add_argument(
        "--config", type=str, default="plan.yaml", help="config for the planner"
    )
    parser.add_argument(
        "--overwrite",
        default=False,
        action="store_true",
        help="force overwrite existing results",
    )
    args = parser.parse_args()

    # with open(args.config, "r") as config:
    #     cfg = yaml.load(config, Loader=yaml.FullLoader)
    # cfg = Namespace(**cfg)

    cfg = OmegaConf.load(args.config)

    main(cfg, overwrite=args.overwrite)
