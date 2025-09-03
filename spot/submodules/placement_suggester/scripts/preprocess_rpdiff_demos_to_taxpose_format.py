
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


from placement_suggester.dataset.rpdiff_data_module import RpDiffDataModule

def main(cfg):
    print()
    print(f"Loading files from:\n{cfg.rpdiff_obj_config}")

    save_locations = [
        f"{cfg.data_folder}/train_rpdiff_preprocessed_{cfg.rpdiff_obj_config}/renders",
        f"{cfg.data_folder}/test_rpdiff_preprocessed_{cfg.rpdiff_obj_config}/renders",
    ]
    print(f"Saving files to:\n{save_locations}")

    for save_location in save_locations:
        # Create folder for save_location if it doesn't exist
        if not os.path.exists(save_location):
            os.makedirs(save_location, exist_ok=True)
        
        if len(glob.glob(f"{save_location}/*")) > 0:
            print(f"\nWARNING: The save location is is not empty: {save_location}\n")
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

    add_multi_obj_mesh_file = True

    dm = RpDiffDataModule(batch_size=1, obj_config=cfg.rpdiff_obj_config, output_format='taxpose_raw_dataset', add_multi_obj_mesh_file=add_multi_obj_mesh_file)
    dm.setup()

    for save_location, dataloader in zip(save_locations, [dm.train_dataloader(), dm.val_dataloader()]):
        i = 0
        for data_dict in tqdm.tqdm(dataloader):
            for b in range(data_dict['clouds'].shape[0]):
                fname = f"{i}_teleport_obj_points.npz"
                np.savez_compressed(f"{save_location}/{fname}",
                                    clouds=data_dict['clouds'][b],
                                    classes=data_dict['classes'][b],
                                    colors=None,
                                    shapenet_id=None,
                                    multi_obj_mesh_file=data_dict['multi_obj_mesh_file'] if add_multi_obj_mesh_file else None,
                                    multi_obj_final_obj_pose=data_dict['multi_obj_final_obj_pose'] if add_multi_obj_mesh_file else None,
                )
                i += 1
    
    print("done\n")
    
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Data augmentation.')
    parser.add_argument('--data_folder', type=str, 
                        default="/home/jenny/code/placement_suggester/data",
                        help='Folder containing all datasets')
    parser.add_argument('--rpdiff_obj_config', type=str,
                        default="mug-rack-multi",
                        help='Which object config to use from rpdiff')
    parser.add_argument('--overwrite', type=bool, default=False,)
    main(parser.parse_args())