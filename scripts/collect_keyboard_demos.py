import argparse

import hydra

from spot.demo_collection import KeyboardDemo
from spot.pybullet_env.scene_gen.generate_scene_demo import Scene


def main(cfg):
    env = Scene(cfg, gui=True, robot=False)

    demo = KeyboardDemo(cfg, env)

    print("\nLeft = <--")
    print("Right = -->")
    print("Up = Arrow up")
    print("Down = Arrow down")
    print("Roll in, roll out = [ ]")
    print("Pitch in, pitch out = ; ' ")
    print("Yaw in, yaw out = , . ")
    print("Change focus object = /\n")

    demo.collect()


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

    with hydra.initialize(version_base=None, config_path="../configs/blocks"):
        cfg = hydra.compose(args.config)

    main(cfg)