import os
from typing import List, Optional

import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend to prevent figure display
import matplotlib.pyplot as plt
import numpy as np
import torch
from pytorch3d.loss import chamfer_distance
from sklearn.preprocessing import MinMaxScaler, StandardScaler
from torch.utils.data import Dataset
from torch_geometric.data import Data
from spot.utils.pcd_utils import downsample_pcds


def standardize(
    raw_data: List[np.ndarray],
    clip_min: float,
    clip_max: float,
    scaler: Optional[str],
    num_bins: int,
) -> np.ndarray:
    """Helper that clips then (optionally) scales data."""
    raw_data = np.array(raw_data).reshape(-1, 1)
    _, ax = plt.subplots()
    ax.hist(raw_data, bins=num_bins)
    # plt.show() removed to prevent blocking execution

    # Clip outliers
    clipped_data = np.clip(raw_data, a_min=clip_min, a_max=clip_max)
    _, ax = plt.subplots()
    ax.hist(clipped_data, bins=num_bins)
    # plt.show() removed to prevent blocking execution

    # Scale data
    if scaler is not None:
        if scaler == "standard":
            scaler = StandardScaler()
        elif scaler == "minmax":
            scaler = MinMaxScaler()  # Defaults to (0, 1) feature range
        else:
            raise ValueError(f"Scaler {scaler} not supported")

        scaled_data = scaler.fit_transform(clipped_data)
        value = scaler.transform(np.array([3.0]).reshape(1, -1))
        print(f"Value 3.0 has been transformed to {value}")

        _, ax = plt.subplots()
        ax.hist(scaled_data, bins=num_bins)
        # plt.show() removed to prevent blocking execution

    return scaled_data


class PointCloudDataset(Dataset):
    def __init__(self):
        super().__init__()

        # These are filled differently depending on the child class
        self.actions = []
        self.pcds = []
        self.labels = []

    def __len__(self):
        return len(self.pcds)

    def __getitem__(self, idx):
        x = self.actions[idx]  # Point features: action (3 dims)
        pos = self.pcds[idx]
        label = self.labels[idx]

        if type(self.pcds) != torch.Tensor:
            x = torch.from_numpy(x).float()
            pos = torch.from_numpy(pos).float()
            label = torch.from_numpy(label).float()

        return Data(x=x, pos=pos, label=label)


class MDEDataset(PointCloudDataset):
    """
    Custom dataset for point cloud MDE regression model.
    """

    def __init__(
        self,
        dataset_root: str,
        derive_deviation: bool = False,
        mean_centered: bool = False,
        object_ids: Optional[List[int]] = None,
        clip_min: Optional[float] = None,
        clip_max: Optional[float] = None,
        scaler: Optional[str] = None,
    ):
        assert not mean_centered or object_ids is not None

        super().__init__()

        pcds: List[np.ndarray] = []
        deltas: List[np.ndarray] = []
        raw_deviations: List[np.ndarray] = []

        for file in os.listdir(dataset_root):
            filename = os.path.join(dataset_root, file)
            data = np.load(filename)

            start_pcd = data["start_pcd"]
            start_pcd_seg = data["start_pcd_seg"]
            gt_next_pcd = data["gt_next_pcd"]  # TAXPose-D next state

            if not derive_deviation:
                deviation = data["deviation"]
            else:  # Calculate deviation from the non-table points
                next_pcds = data["next_pcds"]  # shape: (n_rollouts, n_points, 3)

                # Remove table points and undo the 100x scaling from data collection
                obj_points_mask = np.isin(start_pcd_seg, object_ids)
                start_pcd_no_table = start_pcd[obj_points_mask] / 100
                gt_next_pcd_no_table = gt_next_pcd[obj_points_mask] / 100
                next_pcds_no_table = next_pcds[:, obj_points_mask, :]

                # Calculating NORMALIZED deviation
                diffs = []
                for next_pcd in next_pcds_no_table:
                    diff = (
                        np.linalg.norm(gt_next_pcd_no_table - next_pcd, axis=1) ** 2
                        + 0.01
                    ) / (
                        np.linalg.norm(
                            gt_next_pcd_no_table - start_pcd_no_table, axis=1
                        )
                        ** 2
                        + 0.01
                    )  # shape: (n_points,)
                    diffs.append(diff)
                arr = np.array(diffs)  # shape: (n_rollouts, n_points)
                deviation = 1 / (arr.shape[0] * arr.shape[1]) * np.sum(arr)

            # Encode action as delta x, delta y, delta z for each point
            delta = gt_next_pcd - start_pcd

            if mean_centered:
                # Mean-center the point cloud on the mean of the non-table points
                center = start_pcd[np.isin(start_pcd_seg, object_ids)].mean(axis=0)
                start_pcd = start_pcd - center

            pcds.append(start_pcd)
            deltas.append(delta)
            raw_deviations.append(deviation)

        deviations = standardize(deviations, clip_min, clip_max, scaler, num_bins=50)

        self.pcds = pcds
        self.actions = deltas
        self.labels = deviations


class MDEClassifierDataset(PointCloudDataset):
    """
    Custom dataset for point cloud MDE classifier.

    Assumes the dataset was calculated with normalized deviation.
    - 0 means valid action
    - 1 means invalid action
    """

    def __init__(
        self,
        dataset_root: str,
        mean_centered: bool = False,
        object_ids: Optional[List[int]] = None,
    ):
        assert (
            not mean_centered or object_ids is not None
        ), "Must provide `object_ids` if using mean-centering"

        super().__init__()

        pcds: List[np.ndarray] = []
        deltas: List[np.ndarray] = []
        gt_labels: List[np.ndarray] = []

        for file in os.listdir(dataset_root):
            filename = os.path.join(dataset_root, file)
            data = np.load(filename)

            start_pcd = data["start_pcd"]
            gt_next_pcd = data["gt_next_pcd"]
            # If normalized deviation is at least 1.0 (a special value),
            # then the action was invalid
            label = int(data["deviation"] >= 1.0)

            delta = gt_next_pcd - start_pcd

            if mean_centered:
                # Mean-center the point cloud on the mean of the non-table points
                start_pcd_seg = data["start_pcd_seg"]
                center = start_pcd[np.isin(start_pcd_seg, object_ids)].mean(axis=0)
                start_pcd = start_pcd - center

            pcds.append(start_pcd)
            deltas.append(delta)
            gt_labels.append(label)

        self.pcds = pcds
        self.actions = deltas
        self.labels = gt_labels


class MDEChamferDistanceDataset(PointCloudDataset):
    """
    Custom dataset for point cloud MDE calculated using Chamfer distance.
    """

    def __init__(
        self,
        dataset_root: str,
        object_ids: List[int],
        eps: float,
        mean_centered: bool = True,
        clip_min: Optional[float] = None,
        clip_max: Optional[float] = None,
        scaler: Optional[str] = None,
        num_points: int = 1024,
    ):
        super().__init__()

        # start_pcds: List[np.ndarray] = []
        # start_pcd_segs: List[np.ndarray] = []
        # gt_next_pcds: List[np.ndarray] = []
        # next_pcds: List[np.ndarray] = []
        # actions: List[np.ndarray] = []

        # for file in os.listdir(dataset_root):
        #     filename = os.path.join(dataset_root, file)
        #     data = np.load(filename)

        #     start_pcd = data["start_pcd"]
        #     pcd_seg = data["start_pcd_seg"]
        #     gt_next_pcd = data["gt_next_pcd"]  # TAXPose-D next state
        #     # Just take the 0th of the next pcds bc it's hard to deal with all of them
        #     next_pcd = (
        #         data["next_pcds"][0] * 100
        #     )  # Forgot to scale these up during data collection

        #     # Remove non-object points
        #     object_mask = np.isin(pcd_seg, object_ids)
        #     start_pcd = start_pcd[object_mask]
        #     gt_next_pcd = gt_next_pcd[object_mask]
        #     next_pcd = next_pcd[object_mask]
        #     pcd_seg = pcd_seg[object_mask]

        #     # Encode action as delta x, delta y, delta z for each point
        #     delta = gt_next_pcd - start_pcd

        #     if mean_centered:
        #         # Mean-center the initial point cloud and translate the other pcds by the same amount
        #         center = start_pcd.mean(axis=0)
        #         start_pcd = start_pcd - center
        #         gt_next_pcd = gt_next_pcd - center
        #         next_pcd = next_pcd - center

        #     start_pcds.append(start_pcd)
        #     start_pcd_segs.append(pcd_seg)
        #     gt_next_pcds.append(gt_next_pcd)
        #     next_pcds.append(next_pcd)
        #     actions.append(delta)

        # # Downsample point clouds and associated data
        # sampled_start_pcds, (
        #     sampled_start_pcd_segs,
        #     sampled_gt_next_pcds,
        #     sampled_next_pcds,
        #     sampled_actions,
        # ) = downsample_pcds(
        #     start_pcds,
        #     [start_pcd_segs, gt_next_pcds, next_pcds, actions],
        #     num_points=num_points,
        # )

        # # Save the data
        # output_dir = (
        #     "/home/amli/research/3DVTAMP/assets/data/three_cubes_dynamics/downsampled"
        # )
        # torch.save(
        #     {
        #         "start_pcds": sampled_start_pcds,
        #         "start_pcd_segs": sampled_start_pcd_segs,
        #         "gt_next_pcds": sampled_gt_next_pcds,
        #         "next_pcds": sampled_next_pcds,
        #         "actions": sampled_actions,
        #     },
        #     os.path.join(output_dir, "data_downsampled.pth"),
        # )

        data = torch.load(os.path.join(dataset_root, "data_downsampled.pth"))

        # NOTE: undo the point cloud scaling back to original scale
        start_pcds = data["start_pcds"] / 100.0
        start_pcd_segs = data["start_pcd_segs"]
        gt_next_pcds = data["gt_next_pcds"] / 100.0
        next_pcds = data["next_pcds"] / 100.0
        actions = data["actions"]

        deviations_filtered: List[np.ndarray] = []
        start_pcds_filtered: List[np.ndarray] = []
        actions_filtered: List[np.ndarray] = []
        # Process each data point: calculate object-wise Chamfer distances
        for start_pcd, pcd_seg, gt_next_pcd, next_pcd, action in zip(
            start_pcds,
            start_pcd_segs,
            gt_next_pcds,
            next_pcds,
            actions,
        ):
            deviation = 0
            for i in object_ids:
                p_init = start_pcd[pcd_seg == i].unsqueeze(0)
                p_gt_next = gt_next_pcd[pcd_seg == i].unsqueeze(0)
                p_next = next_pcd[pcd_seg == i].unsqueeze(0)

                # Normalize by distance between initial and gt next state
                distance = chamfer_distance(p_gt_next, p_next)[0].item()
                scaling = chamfer_distance(p_gt_next, p_init)[0].item()

                deviation += (distance + eps) / (scaling + eps)

            # print("deviation:", deviation)
            # Filter out data points where no objects move
            if np.allclose(deviation, 3.0):
                continue

            deviations_filtered.append(np.array(deviation))
            start_pcds_filtered.append(start_pcd)
            actions_filtered.append(action)

        assert (
            len(deviations_filtered)
            == len(start_pcds_filtered)
            == len(actions_filtered)
        )

        deviations = standardize(
            deviations_filtered, clip_min, clip_max, scaler, num_bins=50
        )
        deviations = torch.from_numpy(np.stack(deviations)).float()
        start_pcds_filtered = torch.stack(start_pcds_filtered).float()
        actions_filtered = torch.stack(actions_filtered).float()

        self.pcds = start_pcds_filtered
        self.actions = actions_filtered
        self.labels = deviations


class MDETableBussingDataset(PointCloudDataset):
    """
    Dataset for MDE model for the table bussing task.

    Uses Chamfer distance to calculate true deviations.
    """

    def __init__(
        self,
        dataset_root: str,
        object_ids: List[int],
        eps: float,
        clip_min: Optional[float] = None,
        clip_max: Optional[float] = None,
        scaler: Optional[str] = None,
        num_points: int = 1024,
    ):
        super().__init__()

        # start_pcds: List[np.ndarray] = []
        # start_pcd_segs: List[np.ndarray] = []
        # gt_next_pcds: List[np.ndarray] = []
        # next_pcds: List[np.ndarray] = []
        # next_pcd_segs: List[np.ndarray] = []
        # actions: List[np.ndarray] = []

        # for file in os.listdir(dataset_root):
        #     filename = os.path.join(dataset_root, file)
        #     data = np.load(filename)

        #     start_pcd = data["start_pcd"]
        #     start_pcd_seg = data["start_pcd_seg"]
        #     gt_next_pcd = data["predicted_pcd"]  # From TAXPose-D
        #     next_pcd = data["actual_pcd"]
        #     next_pcd_seg = data["actual_pcd_seg"]

        #     # Remove non-object points
        #     start_pcd = start_pcd[np.isin(start_pcd_seg, object_ids)]
        #     gt_next_pcd = gt_next_pcd[np.isin(start_pcd_seg, object_ids)]
        #     start_pcd_seg = start_pcd_seg[np.isin(start_pcd_seg, object_ids)]
        #     next_pcd = next_pcd[np.isin(next_pcd_seg, object_ids)]
        #     next_pcd_seg = next_pcd_seg[np.isin(next_pcd_seg, object_ids)]

        #     # Scale point clouds up by 100x
        #     # start_pcd *= 100
        #     # gt_next_pcd *= 100
        #     # next_pcd *= 100

        #     # Encode action as delta x, delta y, delta z for each point
        #     action = gt_next_pcd - start_pcd

        #     # Mean-center
        #     center = start_pcd.mean(axis=0)
        #     start_pcd_centered = start_pcd - center
        #     next_pcd_centered = next_pcd - center
        #     gt_next_pcd_centered = gt_next_pcd - center

        #     start_pcds.append(start_pcd_centered)
        #     start_pcd_segs.append(start_pcd_seg)
        #     gt_next_pcds.append(gt_next_pcd_centered)
        #     next_pcds.append(next_pcd_centered)
        #     next_pcd_segs.append(next_pcd_seg)
        #     actions.append(action)

        # # Downsample point clouds and associated data
        # sampled_start_pcds, (
        #     sampled_start_pcd_segs,
        #     sampled_gt_next_pcds,
        #     sampled_actions,
        # ) = downsample_pcds(
        #     start_pcds,
        #     [start_pcd_segs, gt_next_pcds, actions],
        #     num_points=num_points,
        # )
        # sampled_next_pcds, (sampled_next_pcd_segs,) = downsample_pcds(
        #     next_pcds,
        #     [next_pcd_segs],
        #     num_points=num_points,
        # )

        # output_dir = (
        #     "/home/amli/research/3DVTAMP/assets/data/mde_table_bussing2_not_scaled"
        # )
        # torch.save(
        #     {
        #         "start_pcds": sampled_start_pcds,
        #         "start_pcd_segs": sampled_start_pcd_segs,
        #         "gt_next_pcds": sampled_gt_next_pcds,
        #         "next_pcds": sampled_next_pcds,
        #         "next_pcd_segs": sampled_next_pcd_segs,
        #         "actions": sampled_actions,
        #     },
        #     os.path.join(output_dir, "data.pth"),
        # )

        data = torch.load(os.path.join(dataset_root, "data_downsampled.pth"))

        print("data:", data.keys())

        sampled_start_pcds = data["start_pcds"]
        sampled_start_pcd_segs = data["start_pcd_segs"]
        sampled_gt_next_pcds = data["gt_next_pcds"]
        sampled_next_pcds = data["next_pcds"]
        sampled_next_pcd_segs = data["start_pcd_segs"]
        sampled_actions = data["actions"]

        deviations = []
        # Process each data point: calculate object-wise Chamfer distances
        for start_pcd, gt_next_pcd, pcd_seg, next_pcd, next_pcd_seg in zip(
            sampled_start_pcds,
            sampled_gt_next_pcds,
            sampled_start_pcd_segs,
            sampled_next_pcds,
            sampled_next_pcd_segs,
        ):
            deviation = 0
            for i in object_ids:
                p_init = start_pcd[pcd_seg == i].unsqueeze(0)
                p_gt_next = gt_next_pcd[pcd_seg == i].unsqueeze(0)
                p_next = next_pcd[next_pcd_seg == i].unsqueeze(0)

                # Normalize by distance between initial and gt next state
                distance = chamfer_distance(p_gt_next, p_next)[0].item()
                scaling = chamfer_distance(p_gt_next, p_init)[0].item()

                d = (distance + eps) / (scaling + eps)
                # print(d)
                deviation += d
            # print("deviation:", deviation)
            deviations.append(np.array(deviation))

        deviations = standardize(deviations, clip_min, clip_max, scaler, num_bins=20)
        deviations = torch.from_numpy(np.stack(deviations)).float()

        self.pcds = sampled_start_pcds
        self.actions = sampled_actions
        self.labels = deviations


class MDETableBussingClassifierDataset(PointCloudDataset):
    """
    Dataset for MDE model for the table bussing task.

    Uses Chamfer distance to calculate true deviations.
    """

    def __init__(
        self,
        dataset_root: str,
        mean_centered: bool = True,
        object_ids: Optional[List[int]] = None,
        num_points: int = 1024,
    ):
        assert not mean_centered or object_ids is not None

        super().__init__()

        pcds: List[np.ndarray] = []
        actions: List[np.ndarray] = []
        labels: List[np.ndarray] = []

        for file in os.listdir(dataset_root):
            filename = os.path.join(dataset_root, file)
            data = np.load(filename)

            start_pcd = data["start_pcd"]
            start_pcd_seg = data["start_pcd_seg"]
            predicted_pcd = data["predicted_pcd"]  # From TAXPose-D
            label = 1 - int(data["label"])

            # Encode action as delta x, delta y, delta z for each point
            action = predicted_pcd - start_pcd

            if mean_centered:
                # Mean-center the point cloud on the mean of the non-table points
                center = start_pcd[np.isin(start_pcd_seg, object_ids)].mean(axis=0)
                start_pcd = start_pcd - center

            pcds.append(start_pcd)
            actions.append(action)
            labels.append(label)

        labels = torch.from_numpy(np.stack(labels)).float()

        # Downsample point clouds and associated data
        sampled_pcds, (sampled_actions,) = downsample_pcds(
            pcds, [actions], num_points=num_points
        )

        self.pcds = sampled_pcds
        self.actions = sampled_actions
        self.labels = labels
