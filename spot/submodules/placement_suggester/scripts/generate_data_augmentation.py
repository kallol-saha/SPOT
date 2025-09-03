
import sys
import os
import torch

os.environ['CUDA_DEVICE_ORDER']='PCI_BUS_ID'
os.environ['CUDA_VISIBLE_DEVICES']='0'

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import torch.nn.functional as F

import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

from placement_suggester.utils.env_mod_utils import get_random_rack_demo

import glob
import tqdm

ACTION_CLASS = 0
ANCHOR_CLASS = 1
GRIPPER_CLASS = 2

def main(cfg):
    defaults = {
        'train_default': {
            'dataset_root': "/media/jenny/cubbins-archive/jwang_datasets/home_backup/jwang/code/placement_suggester/data/train_data/renders",
        },
        'test_default': {
            'dataset_root': "/media/jenny/cubbins-archive/jwang_datasets/home_backup/jwang/code/placement_suggester/data/test_data/renders",
        }
    }

    if cfg.dataset_root in defaults.keys():
        dataset_root = defaults[cfg.dataset_root]['dataset_root']

    dataset_root_parts = dataset_root.split('/')
    tag = "aug"
    if cfg.no_transform_base:
        tag += "_easy"
    save_location = f"{'/'.join(dataset_root_parts[:-1])}_{tag}/{dataset_root_parts[-1]}"

    print()
    print("Ensure the base dataset is for 1-racks, not 2-racks.")
    print()
    print(f"Loading files from:\n{dataset_root}")
    print(f"Saving files to:\n{save_location}")
    
    fnames = glob.glob(f"{dataset_root}/*")

    if len(glob.glob(f"{save_location}/*")) > 0:
        print(f"\nWARNING: The save location is is not empty.\n")
        if cfg.overwrite:
            print("Overwriting...")
            for f in glob.glob(f"{save_location}/*"):
                os.remove(f)
        else:
            ans = input("Overwrite? (y/n): ")
            while ans not in ['y', 'n']:
                ans = input("Overwrite? (y/n): ")
            if ans == 'n':
                print("Exiting...")
                sys.exit(0)
            else:
                print("Overwriting...")
                for f in glob.glob(f"{save_location}/*"):
                    os.remove(f)

    mugid_namelen = np.ceil(np.log10(max([int(os.path.basename(fn).split("_")[0]) for fn in fnames]))).astype(int)
    aug_namelen = np.ceil(np.log10(cfg.augs_per_demo)).astype(int)

    for data_fn in tqdm.tqdm(fnames):
        all_data = np.load(data_fn)

        assert np.unique(all_data['classes']).tolist() == [ACTION_CLASS, ANCHOR_CLASS, GRIPPER_CLASS], \
                f"There are different classes than expected: {np.unique(all_data['classes']).tolist()}"
        
        gripper_points_base = torch.tensor(all_data['clouds'][all_data['classes'] == GRIPPER_CLASS]).float()[None]
        action_points_base = torch.tensor(all_data['clouds'][all_data['classes'] == ACTION_CLASS]).float()[None]
        anchor_points_base = torch.tensor(all_data['clouds'][all_data['classes'] == ANCHOR_CLASS]).float()[None]
        
        for i in range(cfg.augs_per_demo):
            batch_points_gripper, batch_points_action, batch_points_anchor1, batch_points_anchor2, debug = \
                    get_random_rack_demo(gripper_points_base, action_points_base, anchor_points_base, transform_base=not cfg.no_transform_base, rot_sample_method=cfg.rot_sample_method)

            # save
            new_clouds = np.concatenate([
                            batch_points_gripper[0].numpy(),
                            batch_points_action[0].numpy(),
                            batch_points_anchor1[0].numpy(),
                            batch_points_anchor2[0].numpy(),
                        ])
            new_classes = np.concatenate([
                            all_data['classes'][all_data['classes'] == GRIPPER_CLASS],
                            all_data['classes'][all_data['classes'] == ACTION_CLASS],
                            all_data['classes'][all_data['classes'] == ANCHOR_CLASS],
                            all_data['classes'][all_data['classes'] == ANCHOR_CLASS],
                        ])
            new_colors = np.concatenate([
                            all_data['colors'][all_data['classes'] == GRIPPER_CLASS],
                            all_data['colors'][all_data['classes'] == ACTION_CLASS],
                            all_data['colors'][all_data['classes'] == ANCHOR_CLASS],
                            all_data['colors'][all_data['classes'] == ANCHOR_CLASS],
                        ])
            new_shapenet_id = all_data['shapenet_id']

            fname_parts = data_fn.split('/')[-1].split('_')
            mugid = "1" + f'{fname_parts[0]}'.rjust(mugid_namelen, '0') + f'{i}'.rjust(aug_namelen, '0')
            fname = "_".join([mugid] + fname_parts[1:])
            np.savez_compressed(f"{save_location}/{fname}",
                                clouds=new_clouds,
                                classes=new_classes,
                                colors=new_colors,
                                shapenet_id=new_shapenet_id,
            )
    print("done\n")
    
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Data augmentation.')
    parser.add_argument('--dataset_root', type=str, 
                        default="train_default",
                        help='This is a dataset for 1-rack demonstrations')
    parser.add_argument('--augs_per_demo', type=int, default=100,)
    parser.add_argument('--overwrite', type=bool, default=False,)
    parser.add_argument('--no_transform_base', action='store_true',
                        help='Whether or not to transform the base demo. If no transformation is applied, this is a simpler environment.')
    parser.add_argument('--rot_sample_method', type=str, 
                        default="axis_angle",
                        help='How to sample random rotations for randomizing the environment')
    main(parser.parse_args())