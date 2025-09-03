import argparse
import os

import numpy as np
from omegaconf import OmegaConf

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Evaluate the planning algorithm")
    parser.add_argument(
        "--config",
        type=str,
        default="execute_real_world_plan.yaml",
        help="config for the planner",
    )
    args = parser.parse_args()

    # with open(args.config, "r") as config:
    #     cfg = yaml.load(config, Loader=yaml.FullLoader)
    # cfg = Namespace(**cfg)
    cfg = OmegaConf.load(args.config)

    plan_path = os.path.join(cfg.plan_folder, "output_plan.npz")
    plan_npz = np.load(plan_path)

    with open("obj_id_seq.txt", "w") as file:
        for i in range(len(plan_npz["object_order"])):
            object_id = plan_npz["object_order"][i]
            # Specify the file name

            # Write a number followed by a newline
            file.write(str(object_id) + "\n")
            # print(object_id)

# plan_npz = np.load("transformation_npy/5/output_plan.npz") # (to do) change path in the loop

# with open("obj_id_seq.txt", 'w') as file:
#     pass
