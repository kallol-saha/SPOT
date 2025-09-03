"""
Pre-generate and save initial states (point clouds and simulator states) for
benchmarking planning and execution.
"""

import argparse
import os
import shutil

import cv2
import hydra
import numpy as np

from spot.pybullet_env.scene_gen.generate_scene import Scene
from spot.suggester_utils import set_seed
from spot.utils.data_collection_utils import (
    _sample_initial_piles,
    _sample_state_from_piles,
)


def _generate_initial_states(env, rng, N):
    initial_states = []
    for _ in range(N):
        # Set random initial state
        piles = _sample_initial_piles(env, rng, shuffle=False)
        state = _sample_state_from_piles(piles, rng)
        initial_states.append(state)
    return initial_states


def main(cfg, overwrite: bool = False):
    # Reproducibility:
    set_seed(cfg.seed)
    # NOTE that the torch seed gets reset in the get_data() function inside
    # TestPointCloudDataset, should be fine for now.

    # Prepare the output directory for the benchmark:
    output_dir = os.path.join(cfg.eval_folder_location, cfg.benchmark_name)
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
        os.path.join("configs", str(args.config)),
        os.path.join(output_dir, "benchmark.yaml"),
    )

    # Load simulation
    env = Scene(cfg, seed=cfg.seed, gui=cfg.gui)
    rng = np.random.default_rng(seed=cfg.seed)

    # Generate tasks: (initial state, goal heuristic)
    initial_states = _generate_initial_states(env, rng, cfg.num_initial_states)
    # tasks = product(initial_states, heuristics)

    i = 0
    for initial_state in initial_states:
        # Set initial state
        env.set_initial_state(initial_state)
        env.reset()

        # Get initial pcd and segmentation
        initial_pcd, initial_pcd_seg, _ = env.get_observation()
        initial_image, _, _ = env.camera_list[
            0
        ].capture()  # NOTE: I also want to save the image so I can view the what the example is easily
        initial_image = cv2.cvtColor(initial_image, cv2.COLOR_RGB2BGR)
        # print(f"Goal is {heuristic.name}")
        # if heuristic(cfg, initial_pcd, initial_pcd_seg) == 0:
        #     # Goal already reached, skip
        #     continue

        # Save initial point cloud
        # NOTE from Kallol: I am changing this to match the eval format of number.npz
        np.savez(
            os.path.join(output_dir, f"{i}.npz"),
            initial_pcd=initial_pcd,
            initial_pcd_seg=initial_pcd_seg,
            # heuristic=heuristic,
        )

        # Save initial state data for use in execution
        np.savez(
            os.path.join(output_dir, f"{i}_initial_state.npz"),
            initial_state=initial_state,
        )

        cv2.imwrite(os.path.join(output_dir, f"{i}.jpg"), initial_image)

        i += 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Save initial states for planning and execution benchmarking"
    )
    parser.add_argument(
        "--config",
        type=str,
        default="benchmark_blocks.yaml",
        help="config for the planner",
    )
    parser.add_argument(
        "--overwrite",
        default=False,
        action="store_true",
        help="force overwrite existing output",
    )
    args = parser.parse_args()

    with hydra.initialize(version_base=None, config_path="configs"):
        cfg = hydra.compose(args.config)

    main(cfg, overwrite=args.overwrite)
