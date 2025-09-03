import os

import lightning as L
import numpy as np
import torch
from pytorch3d.ops import sample_farthest_points
from torch.utils.data import DataLoader, Dataset


class PointCloudDataset(Dataset):
    def __init__(
        self,
        dataset_folder,
        pzY_input_dims=4,
        num_points=1024,
        rotation_variance=np.pi,
        translation_variance=0.5,
        angle_degree=180,
        downsample_type="fps",
        rot_sample_method="random_flat_upright",
        num_suggestions=4,
        seed=0,
    ):
        self.num_points = num_points
        self.dataset_folder = dataset_folder
        self.files = [f for f in os.listdir(self.dataset_folder) if f.endswith(".npz")]
        self.pzY_input_dims = pzY_input_dims
        # Path('/home/bokorn/src/ndf_robot/notebooks')
        self.rot_var = rotation_variance
        self.trans_var = translation_variance
        self.angle_degree = angle_degree
        self.rot_sample_method = rot_sample_method
        self.downsample_type = downsample_type

        self.num_suggestions = num_suggestions
        self.dataset_size = len(self.files)
        self.seed = seed

    def __getitem__(self, idx):
        file_path = os.path.join(self.dataset_folder, self.files[idx])
        return self.get_data(file_path, seed=self.seed)

    def __len__(self):
        return self.dataset_size

    def downsample_pcd(self, points, type="fps"):
        points = points.unsqueeze(0)
        if type == "fps":
            return sample_farthest_points(
                points, K=self.num_points, random_start_point=True
            )
        elif type == "random":
            random_idx = torch.randperm(points.shape[1])[: self.num_points]
            return points[:, random_idx], random_idx
        elif type.startswith("random_"):
            prob = float(type.split("_")[1])
            if np.random.random() < prob:
                return sample_farthest_points(
                    points, K=self.num_points, random_start_point=True
                )
            else:
                random_idx = torch.randperm(points.shape[1])[: self.num_points]
                return points[:, random_idx], random_idx

    def load_data(self, file_path):
        point_data = np.load(file_path, allow_pickle=True)

        points_raw_np = point_data["clouds"]
        mask_raw_np = point_data["masks"]
        moved_next = point_data["moved_next"]

        points_mean_np = points_raw_np.mean(axis=0)
        points_np = points_raw_np - points_mean_np

        points = torch.from_numpy(points_np).float()
        mask = torch.from_numpy(mask_raw_np).long().unsqueeze(1)
        moved_next = torch.from_numpy(moved_next).long()

        points = torch.cat([points, mask], axis=-1)
        return points, moved_next, points_mean_np

    def get_data(self, file_path, seed=0):
        points, moved_next, mean_point = self.load_data(file_path)
        torch.manual_seed(seed)
        points, _ = self.downsample_pcd(points, type=self.downsample_type)

        data = {
            "clouds": points.squeeze(0),
            "moved_next": moved_next,
            "mean_point": mean_point,
        }

        return data


class PointCloudDataModule(L.LightningDataModule):
    def __init__(self, batch_size, num_workers, train_folder, val_folder):
        super().__init__()
        self.batch_size = batch_size
        self.num_workers = num_workers
        self.train_folder = train_folder
        self.val_folder = val_folder

    def setup(self, stage=None):
        self.train_dataset = PointCloudDataset(self.train_folder)
        self.val_dataset = PointCloudDataset(self.val_folder)

    def train_dataloader(self):
        return DataLoader(
            self.train_dataset,
            batch_size=self.batch_size,
            shuffle=True,
            num_workers=self.num_workers,
        )

    def val_dataloader(self):
        return DataLoader(
            self.val_dataset,
            batch_size=self.batch_size,
            shuffle=False,
            num_workers=self.num_workers,
        )
