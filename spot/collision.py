from typing import List

import numpy as np
from spot.utils.pcd_utils import transform_pcd


class CollisionChecker:
    def __init__(self, cfg):
        self.cfg = cfg
        self.pick_threshold = cfg.pick_threshold
        self.drop_threshold = cfg.drop_threshold

        if cfg.extrinsics_file is not None:
            self.T_cam_to_world = np.load(self.cfg.extrinsics_file)["T"]
        else:
            self.T_cam_to_world = np.eye(4)

        # Define the voxel grid:
        self.voxel_grid = np.zeros(
            (self.cfg.x_width, self.cfg.y_width, self.cfg.z_width), dtype=np.int32
        )
        self.lower_limits = np.array(
            [self.cfg.x_lower, self.cfg.y_lower, self.cfg.z_lower]
        )
        self.upper_limits = np.array(
            [self.cfg.x_upper, self.cfg.y_upper, self.cfg.z_upper]
        )

        self.voxel_size = np.abs(self.upper_limits - self.lower_limits) / np.array(
            self.voxel_grid.shape
        )

        self.remove_outliers = cfg.remove_outliers

    def get_voxel_points_and_mask(self, pcd_input: np.ndarray):
        # drop_diff = self.cfg.drop_height - np.min(pcd, axis = 0)[-1]
        pcd = pcd_input.copy()
        pcd[:, -1] = pcd[:, -1] + self.cfg.drop_height

        voxel_indices = np.floor((pcd - self.lower_limits) / self.voxel_size).astype(
            np.int32
        )

        # Only keep points that are within the voxel grid:
        mask = np.all(
            (voxel_indices >= 0) & (voxel_indices < self.voxel_grid.shape), axis=1
        )
        voxel_indices = voxel_indices[mask]

        # Calculate the number of points in each voxel:
        voxel_points = self.voxel_grid.copy()
        np.add.at(voxel_points, tuple(voxel_indices.T), 1)

        # Calculate a mask for whether a voxel has points or not:
        voxel_mask = (voxel_points > 0).astype(np.int32)

        return voxel_points, voxel_mask

    def sweep_up_pcd(self, pcd):
        dist_to_sweep = np.abs(self.upper_limits[-1] - np.max(pcd[:, -1]))
        sweep_steps = np.ceil(dist_to_sweep / self.voxel_size[-1]).astype(np.int32)
        sweep_steps += 1  # To add the current position of the point cloud too.

        # For adding to z-axis of the point cloud to sweep it up:
        sweep_up = np.repeat(np.arange(sweep_steps), pcd.shape[0]) * self.voxel_size[-1]

        # Form the swept point cloud:
        pcd_swept = np.tile(pcd, (sweep_steps, 1))
        pcd_swept[:, -1] = pcd_swept[:, -1] + sweep_up

        return pcd_swept

    def is_colliding(
        self,
        pcd: np.ndarray,
        pcd_seg: np.ndarray,
        obj_ids: List[int],
        moved_obj: int,
        mode: str = "pick",
    ):
        """
        Check if the object is colliding with the environment.
        Input:
            pcd: Point cloud of the scene
            pcd_seg: Segmentation of the scene
            obj_ids: List of object ids
            moved_obj: The object id that is being moved
        Output:
            bool: Whether the object is colliding with the environment
            collision_ratio: The ratio of points that are colliding
        """
        # We only care about the movable objects
        current_pcd = pcd[np.isin(pcd_seg, obj_ids)]
        pcd_seg = pcd_seg[np.isin(pcd_seg, obj_ids)]

        # Transform pcd to world frame with extrinsic matrix:
        current_pcd = transform_pcd(current_pcd, self.T_cam_to_world)

        action_mask = pcd_seg == moved_obj

        anchor_pcd = current_pcd[~action_mask]
        action_pcd = current_pcd[action_mask]

        # Remove outliers from action pcd here.
        # action_pcd, _ = remove_outliers(action_pcd,
        #                              inlier_ratio=self.cfg.inlier_ratio,
        #                              radius=self.cfg.radius)

        action_pcd = self.sweep_up_pcd(action_pcd)

        action_voxel_points, action_voxel_mask = self.get_voxel_points_and_mask(
            action_pcd
        )
        _, anchor_voxel_mask = self.get_voxel_points_and_mask(anchor_pcd)

        # Number of points colliding currently:
        collision_points = (action_voxel_points) * action_voxel_mask * anchor_voxel_mask

        total_collision_points = np.sum(collision_points)
        collision_ratio = total_collision_points / action_pcd.shape[0]

        if mode == "pick":
            return (collision_ratio > self.pick_threshold), collision_ratio
        elif mode == "drop":
            return (collision_ratio > self.drop_threshold), collision_ratio
