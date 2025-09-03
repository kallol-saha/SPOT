import glob
import os
from dataclasses import dataclass
import copy
from typing import Optional, Sequence, Tuple

import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d


class KeyboardDemo:
    def __init__(self, args, sim, mode = "train"):
        self.sim = sim
        self.mode = mode

        # Prepare data folder:
        self.folder_path = args["demo_folder"] + args["exp_name"] + "/"
        os.makedirs(self.folder_path, exist_ok=True)
        os.makedirs(self.folder_path + "placement_suggester/train/", exist_ok=True)
        os.makedirs(self.folder_path + "object_suggester/train/", exist_ok=True)
        os.makedirs(self.folder_path + "placement_suggester/test/", exist_ok=True)
        os.makedirs(self.folder_path + "object_suggester/test/", exist_ok=True)

        self.placement_suggester_train_demos = len(os.listdir(self.folder_path + "placement_suggester/train/"))
        self.object_suggester_train_demos = len(os.listdir(self.folder_path + "object_suggester/train/"))
        self.placement_suggester_test_demos = len(os.listdir(self.folder_path + "placement_suggester/test/"))
        self.object_suggester_test_demos = len(os.listdir(self.folder_path + "object_suggester/test/"))

    def plot_pcd(self, pcd, pcd_seg):
        seg_ids = np.unique(pcd_seg)
        n = len(seg_ids)
        cmap = plt.get_cmap("tab10")
        id_to_color = {uid: cmap(i / n)[:3] for i, uid in enumerate(seg_ids)}

        colors = np.array([id_to_color[seg_id] for seg_id in pcd_seg])
        print("Seg IDs = ", seg_ids)
        print("Colors = ", id_to_color)

        pts_vis = o3d.geometry.PointCloud()
        pts_vis.points = o3d.utility.Vector3dVector(pcd)
        pts_vis.colors = o3d.utility.Vector3dVector(colors)
        o3d.visualization.draw_geometries([pts_vis])

    def save_data(self, moved_obj, data_type = "placement_suggester"):
        pcd, pcd_seg, rgb = self.sim.get_fused_pcd()
        mask = (pcd_seg != 0) & (
            pcd_seg != self.sim.robot_id
        )  # Remove background and robot

        # !!! I am hardcoding here to crop the table !!!
        table_mask = ((np.abs(pcd[:, 0]) < 0.2) & (np.abs(pcd[:, 1]) < 0.2)) | (
            pcd_seg != self.sim.objects["table"]
        )
        mask = table_mask & mask
        # !!! End of hard-coding !!!

        pcd = pcd[mask]
        pcd_seg = pcd_seg[mask]
        rgb = rgb[mask]

        # !!! I am hardcoding here to remove points from the table !!!
        n = 5
        table_indices = np.where(pcd_seg == self.sim.objects["table"])[0]
        keep_indices = table_indices[::n]
        mask = np.ones(pcd_seg.shape, dtype=bool)
        mask[table_indices] = False
        mask[keep_indices] = True
        pcd = pcd[mask]
        pcd_seg = pcd_seg[mask]
        rgb = rgb[mask]
        # !!! End of hard-coding !!!

        obj_mask = np.where(pcd_seg == moved_obj, 0, 1)

        # Make the fixed object segmentation IDs as 0
        pcd_seg = np.where(np.isin(pcd_seg, self.sim.fixed_obj_ids), 0, pcd_seg)

        self.plot_pcd(pcd, obj_mask)

        # np.savez(
        #     self.demo_folder + "pcd_" + str(self.steps) + ".npz",
        #     clouds=pcd,
        #     masks=pcd_seg,
        #     classes=obj_mask,
        # )

        if data_type == "placement_suggester":

            if self.mode == "train":
                np.savez(
                    self.folder_path + "placement_suggester/train/" + str(self.placement_suggester_train_demos) + "_teleport_obj_points.npz",
                    clouds=pcd,
                    masks=pcd_seg,
                    classes=obj_mask,
                )

                self.placement_suggester_train_demos += 1

            elif self.mode == "test":
                np.savez(
                    self.folder_path + "placement_suggester/test/" + str(self.placement_suggester_test_demos) + "_teleport_obj_points.npz",
                    clouds=pcd,
                    masks=pcd_seg,
                    classes=obj_mask,
                )

                self.placement_suggester_test_demos += 1

        elif data_type == "object_suggester":
            
            if self.mode == "train":
                np.savez(
                    self.folder_path + "object_suggester/train/" + str(self.object_suggester_train_demos) + "_teleport_obj_points.npz",
                    clouds=pcd,
                    masks=pcd_seg,
                    classes=obj_mask,
                )
                self.object_suggester_train_demos += 1

            elif self.mode == "test":
                np.savez(
                    self.folder_path + "object_suggester/test/" + str(self.object_suggester_test_demos) + "_teleport_obj_points.npz",
                    clouds=pcd,
                    masks=pcd_seg,
                    classes=obj_mask,
                )

                self.object_suggester_test_demos += 1
    
    def collect(self):
        _ = input("Press Enter to Start collecting train demos")

        print("\nReady to go!")
        prev_state = self.sim.client_id.saveState()

        while True:
            moved_obj = self.sim.control_objects()
            if moved_obj:
                state = self.sim.client_id.saveState()
                save = input(
                    "Do you want to save this transition? (y/n). Type 'R' to reset "
                )
                if save == "R":
                    self.sim.reset(prev_state)
                    state = self.sim.client_id.saveState()
                    print("\nReady to go again!")
                if save == "y" or save == "Y":
                    self.save_data(moved_obj, data_type = "placement_suggester")
                    self.sim.reset(prev_state)
                    self.save_data(moved_obj, data_type = "object_suggester")
                    self.sim.reset(state)
                prev_state = copy.deepcopy(state)
