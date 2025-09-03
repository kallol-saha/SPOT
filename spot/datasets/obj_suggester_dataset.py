import os
from typing import List

import numpy as np
import torch
from torch.utils.data import Dataset
from torch_geometric.data import Data
from spot.utils.pcd_utils import transform_pcd_tensor


def random_se3(batch_size=1, device=None, t_limit=0.2):
    """
    Generate a batch of random SE(3) transforms as 4x4 homogeneous transformation matrices.

    Args:
        batch_size (int): Number of transforms to generate
        device (torch.device, optional): Device to place the tensors on

    Returns:
        torch.Tensor: Batch of SE(3) transforms with shape (batch_size, 4, 4)
    """
    # Generate random Euler angles (in radians) for the batch
    roll = torch.rand(batch_size, device=device) * 2 * np.pi - np.pi  # [-π, π]
    pitch = torch.rand(batch_size, device=device) * 2 * np.pi - np.pi  # [-π, π]
    yaw = torch.rand(batch_size, device=device) * 2 * np.pi - np.pi  # [-π, π]

    # Create batched rotation matrices for each axis
    zeros = torch.zeros(batch_size, device=device)
    ones = torch.ones(batch_size, device=device)

    # Rx = rotation around x-axis
    Rx = torch.stack(
        [
            torch.stack([ones, zeros, zeros], dim=1),
            torch.stack([zeros, torch.cos(roll), -torch.sin(roll)], dim=1),
            torch.stack([zeros, torch.sin(roll), torch.cos(roll)], dim=1),
        ],
        dim=2,
    )

    # Ry = rotation around y-axis
    Ry = torch.stack(
        [
            torch.stack([torch.cos(pitch), zeros, torch.sin(pitch)], dim=1),
            torch.stack([zeros, ones, zeros], dim=1),
            torch.stack([-torch.sin(pitch), zeros, torch.cos(pitch)], dim=1),
        ],
        dim=2,
    )

    # Rz = rotation around z-axis
    Rz = torch.stack(
        [
            torch.stack([torch.cos(yaw), -torch.sin(yaw), zeros], dim=1),
            torch.stack([torch.sin(yaw), torch.cos(yaw), zeros], dim=1),
            torch.stack([zeros, zeros, ones], dim=1),
        ],
        dim=2,
    )

    # Combine rotations
    R = torch.bmm(torch.bmm(Rz, Ry), Rx)

    # Generate random translations in [-t_limit, t_limit]
    t = t_limit * (2 * torch.rand(batch_size, 3, device=device) - 1)

    # Create batched homogeneous transformation matrices
    transform = torch.eye(4, device=device).repeat(batch_size, 1, 1)
    transform[:, :3, :3] = R
    transform[:, :3, 3] = t

    return transform


class ObjectSuggesterDataset(Dataset):
    """
    Dataset for object suggester.
    """

    def __init__(
        self,
        dataset_root: str,
        object_ids: List[int],
        num_points: int = 1024,
    ):
        # super().__init__()

        # pcds: List[np.ndarray] = []
        # pcd_segs: List[np.ndarray] = []
        # classes: List[np.ndarray] = []
        # for file in os.listdir(dataset_root):
        #     filename = os.path.join(dataset_root, file)
        #     data = np.load(filename)

        #     points = data["clouds"]
        #     pcd_seg = data["masks"]
        #     classes_raw = data["classes"]  # action vs. anchor

        #     # Remove non-object points
        #     points = points[np.isin(pcd_seg, object_ids)]
        #     classes_raw = classes_raw[np.isin(pcd_seg, object_ids)]
        #     pcd_seg = pcd_seg[np.isin(pcd_seg, object_ids)]

        #     # Mean-center
        #     points = points - points.mean(axis=0)

        #     pcds.append(points)
        #     pcd_segs.append(pcd_seg)
        #     classes.append(classes_raw)

        # # Downsample point clouds and associated data
        # sampled_pcds, (sampled_classes, sampled_pcd_segs) = downsample_pcds(
        #     pcds,
        #     [classes, pcd_segs],
        #     num_points=num_points,
        # )

        # output_dir = (
        #     "/home/amli/research/3DVTAMP/assets/data/three_cubes_obj_suggester/train"
        # )
        # torch.save(
        #     {
        #         "pcds": sampled_pcds,
        #         "pcd_segs": sampled_pcd_segs,
        #         "classes": sampled_classes,
        #     },
        #     os.path.join(output_dir, "data.pth"),
        # )

        data = torch.load(os.path.join(dataset_root, "data.pth"))
        sampled_pcds = data["pcds"]
        sampled_classes = data["classes"]
        sampled_pcd_segs = data["pcd_segs"]

        # Apply batch of random transforms
        T = random_se3(batch_size=sampled_pcds.shape[0])  # (N, 4, 4)
        transformed_pcds = transform_pcd_tensor(sampled_pcds, T)

        self.pcds = transformed_pcds
        self.pcd_segs = sampled_pcd_segs
        self.classes = sampled_classes

    def __len__(self):
        return len(self.pcds)

    def __getitem__(self, idx):
        pos = self.pcds[idx]
        label = self.classes[idx]
        # This is only used to compute a loss term, not passed into any network!
        seg_id = self.pcd_segs[idx]

        if type(self.pcds) != torch.Tensor:
            pos = torch.from_numpy(pos).float()
            label = torch.from_numpy(label).float()
            seg_id = torch.from_numpy(seg_id).float()

        return Data(x=label, pos=pos, seg_id=seg_id)


class ObjectSuggesterClassifierDataset(Dataset):
    """
    Dataset for object suggester classifier.
    """

    def __init__(
        self,
        dataset_root: str,
        object_ids: List[int],
    ):
        assert object_ids is not None, "Provide object segmentation ids"

        # This contains pre-downsampled point cloud data
        data = torch.load(os.path.join(dataset_root, "data.pth"))
        pcds = data["pcds"]
        classes = data["classes"]
        pcd_segs = data["pcd_segs"]

        initial_pcds = []
        query_masks = []
        labels = []

        for pcd, mask, pcd_seg in zip(pcds, classes, pcd_segs):
            # Figure out the action object id
            idxs = torch.nonzero(mask == 0, as_tuple=False)
            assert len(idxs) > 0
            action_id = pcd_seg[idxs[0]].item()  # Any index should work
            assert action_id in object_ids

            for seg_id in object_ids:
                initial_pcds.append(pcd)

                # Construct the mask: this is a query for the object
                query_mask = torch.zeros_like(pcd_seg)
                query_mask[pcd_seg == seg_id] = 1.0
                query_masks.append(query_mask)

                if action_id == seg_id:
                    labels.append(torch.ones(1))
                else:
                    labels.append(torch.zeros(1))

        self.initial_pcds = torch.stack(initial_pcds)  # (B, N, 3)
        self.query_masks = torch.stack(query_masks).unsqueeze(-1)  # (B, N, 1)
        self.labels = torch.stack(labels)

    def __len__(self):
        return len(self.initial_pcds)

    def __getitem__(self, idx):
        pos = self.initial_pcds[idx]
        x = self.query_masks[idx]
        label = self.labels[idx]

        return Data(x=x, pos=pos, label=label)
