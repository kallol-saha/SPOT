from copy import deepcopy
from typing import List

import numpy as np
from spot.models.mde_model import load_mde_model


class NoMDE:
    def __init__(self, mde_cfg):
        self.threshold = 0

    def prune(
        self,
        current_pcd: np.ndarray,
        pcd_seg: np.ndarray,
        gt_next_pcd: np.ndarray,
        obj_ids: List[int],
    ):
        return False, 0.0


class MDE:
    def __init__(self, mde_cfg):
        self.active = mde_cfg.active

        if self.active:
            self.model = load_mde_model(
                mde_cfg.folder_name, parent_folder=mde_cfg.weights_folder
            )
            self.mean_centered: bool = mde_cfg.mean_centered
            self.classifier: bool = mde_cfg.classifier
            self.threshold: float = mde_cfg.threshold if not self.classifier else 0.5

    def prune(
        self,
        current_pcd: np.ndarray,
        pcd_seg: np.ndarray,
        gt_next_pcd: np.ndarray,
        obj_ids: List[int],
        remove_table_points: bool = True,
        classifier: bool = False,
    ) -> bool:
        if not self.active:
            return False, 0.0

        prediction = self.predict(
            current_pcd, pcd_seg, gt_next_pcd, obj_ids, remove_table_points, classifier
        )

        return (prediction > self.threshold), prediction

    def predict(
        self,
        current_pcd: np.ndarray,
        pcd_seg: np.ndarray,
        gt_next_pcd: np.ndarray,
        obj_ids: List[int],
        remove_table_points: bool = True,
        scale_pcds: bool = False,
    ) -> float:
        if not self.active:
            return 0.0

        current_pcd = deepcopy(current_pcd)
        gt_next_pcd = deepcopy(gt_next_pcd)
        pcd_seg = deepcopy(pcd_seg)

        if scale_pcds:
            # Scale up by 100x to match training data
            current_pcd = current_pcd * 100
            gt_next_pcd = gt_next_pcd * 100

        if remove_table_points:
            # Remove fixed obj points, e.g. table points
            current_pcd = current_pcd[np.isin(pcd_seg, obj_ids)]
            gt_next_pcd = gt_next_pcd[np.isin(pcd_seg, obj_ids)]
            pcd_seg = pcd_seg[np.isin(pcd_seg, obj_ids)]

        action = gt_next_pcd - current_pcd

        if self.mean_centered:
            # Mean-center the point cloud on the mean of the non-table points
            center = current_pcd[np.isin(pcd_seg, obj_ids)].mean(axis=0)
            current_pcd = current_pcd - center

        return self.model.predict(current_pcd, action)
