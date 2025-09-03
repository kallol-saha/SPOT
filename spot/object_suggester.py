from copy import deepcopy
from typing import List

import numpy as np
from spot.models.object_suggester_classifier import load_model


class ObjectSuggester:
    def __init__(self, obj_cfg):
        self.active = obj_cfg.active

        if self.active:
            self.model = load_model(
                obj_cfg.folder_name, parent_folder=obj_cfg.weights_folder
            )
            # self.mean_centered: bool = mde_cfg.mean_centered
            # self.classifier: bool = mde_cfg.classifier
            # self.threshold: float = mde_cfg.threshold if not self.classifier else 0.5

    def predict(
        self,
        current_pcd: np.ndarray,
        pcd_seg: np.ndarray,
        obj_ids: List[int],
    ) -> float:
        """
        current_pcd: (N, 3) point cloud
        pcd_seg: (N,) segmentation mask
        obj_id: int object id to query
        """

        if not self.active:
            return float("-inf")

        # Scale up by 100x to match training data
        current_pcd = deepcopy(current_pcd)
        pcd_seg = deepcopy(pcd_seg)

        # Remove fixed obj points, e.g. table points
        current_pcd = current_pcd[np.isin(pcd_seg, obj_ids)]
        pcd_seg = pcd_seg[np.isin(pcd_seg, obj_ids)]

        # Mean-center the point cloud on the mean of the non-table points
        center = current_pcd[np.isin(pcd_seg, obj_ids)].mean(axis=0)
        current_pcd = current_pcd - center

        p_array = np.array(
            [
                self.model.predict(current_pcd, pcd_seg, obj)[0, 0]
                .cpu()
                .detach()
                .numpy()
                for obj in obj_ids
            ]
        )

        p_normalized = p_array / p_array.sum()
        obj_probs = {obj: p_normalized[i] for i, obj in enumerate(obj_ids)}

        return obj_probs, p_normalized
