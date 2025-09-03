import hydra
import numpy as np
import torch
from placement_suggester.utils.load_model_utils import load_model
from placement_suggester.utils.se3 import random_se3
from pytorch3d.ops import sample_farthest_points
from torch.nn.functional import softmax
from torch.utils.data import DataLoader, Dataset
from spot.suggester_utils import (
    filter_rotation,
    filter_translation,
    invert_transformation,
)
from spot.utils.pcd_utils import remove_outliers, transform_pcd


class TestPointCloudDataset(Dataset):
    def __init__(
        self,
        point_data,
        cloud_type="final",
        action_class=0,
        anchor_class=1,
        pzY_input_dims=3,
        num_points=1024,
        rotation_variance=np.pi,
        translation_variance=0.5,
        symmetric_class=None,
        angle_degree=180,
        downsample_type="fps",
        action_rot_sample_method="quat_uniform",
        anchor_rot_sample_method="random_flat_upright",
        num_suggestions=4,
        dataset_size=1,
        seed=0,
    ):
        self.num_points = num_points
        self.point_data = point_data
        self.pzY_input_dims = pzY_input_dims
        self.cloud_type = cloud_type
        self.cloud_type_init = "init"
        self.rot_var = rotation_variance
        self.trans_var = translation_variance
        self.action_class = action_class
        self.anchor_class = anchor_class
        self.symmetric_class = symmetric_class  # None if no symmetric class exists
        self.angle_degree = angle_degree
        self.action_rot_sample_method = action_rot_sample_method
        self.anchor_rot_sample_method = anchor_rot_sample_method
        self.downsample_type = downsample_type

        self.num_suggestions = num_suggestions
        self.dataset_size = dataset_size
        self.seed = seed

    def __getitem__(self, idx):
        return self.get_data(idx + self.seed)

    def __len__(self):
        # we can set this to any number since we only return random samples of
        # the data by using a random seed
        return max(self.num_suggestions, self.dataset_size)

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

    def load_data(self, point_data, action_class, anchor_class):
        points_raw_np = point_data["clouds"]
        classes_raw_np = point_data["classes"]
        masks_raw_np = point_data["masks"]

        points_action_np = points_raw_np[classes_raw_np == action_class].copy()
        points_action_mean_np = points_action_np.mean(axis=0)
        points_action_np = points_action_np - points_action_mean_np

        points_anchor_np = points_raw_np[classes_raw_np == anchor_class].copy()
        points_anchor_np = points_anchor_np - points_action_mean_np
        points_anchor_mean_np = points_anchor_np.mean(axis=0)

        mask_action_np = masks_raw_np[classes_raw_np == action_class].copy()
        mask_anchor_np = masks_raw_np[classes_raw_np == anchor_class].copy()

        points_action = torch.from_numpy(points_action_np).float()
        points_anchor = torch.from_numpy(points_anchor_np).float()
        mask_action = torch.from_numpy(mask_action_np).float().unsqueeze(1)
        mask_anchor = torch.from_numpy(mask_anchor_np).float().unsqueeze(1)

        points_action = torch.cat([points_action, mask_action], dim=1)
        points_anchor = torch.cat([points_anchor, mask_anchor], dim=1)

        symmetric_cls = torch.Tensor([])

        return points_action, points_anchor, symmetric_cls, points_action_mean_np

    def get_data(self, seed=0):
        points_action, points_anchor, symmetric_cls, mean_point = self.load_data(
            self.point_data, self.action_class, self.anchor_class
        )
        torch.manual_seed(seed)
        points_action, _ = self.downsample_pcd(points_action, type=self.downsample_type)
        points_anchor, _ = self.downsample_pcd(points_anchor, type=self.downsample_type)

        points_action, mask_action = points_action[:, :, :3], points_action[:, :, 3:]
        points_anchor, mask_anchor = points_anchor[:, :, :3], points_anchor[:, :, 3:]

        T0 = random_se3(
            1,
            rot_var=self.rot_var,
            trans_var=self.trans_var,
            device=points_action.device,
            rot_sample_method=self.action_rot_sample_method,
        )
        T1 = random_se3(
            1,
            rot_var=self.rot_var,
            trans_var=self.trans_var,
            device=points_anchor.device,
            rot_sample_method=self.anchor_rot_sample_method,
        )

        points_action_trans = T0.transform_points(points_action)
        points_anchor_trans = T1.transform_points(points_anchor)

        points_action = torch.cat([points_action, mask_action], axis=-1)[
            :, :, : self.pzY_input_dims
        ]
        points_anchor = torch.cat([points_anchor, mask_anchor], axis=-1)[
            :, :, : self.pzY_input_dims
        ]
        points_action_trans = torch.cat([points_action_trans, mask_action], axis=-1)[
            :, :, : self.pzY_input_dims
        ]
        points_anchor_trans = torch.cat([points_anchor_trans, mask_anchor], axis=-1)[
            :, :, : self.pzY_input_dims
        ]

        data = {
            "points_action": points_action.squeeze(0),
            "points_anchor": points_anchor.squeeze(0),
            "points_action_trans": points_action_trans.cuda().squeeze(0),
            "points_anchor_trans": points_anchor_trans.cuda().squeeze(0),
            "T0": T0.get_matrix().squeeze(0),
            "T1": T1.get_matrix().squeeze(0),
            "symmetric_cls": symmetric_cls,
            "mean_point": mean_point,
        }

        return data


class Suggester:
    """
    Suggestion model class.
    Given an object and an environment, proposes a transformations to be applied to that object
    """

    def __init__(self, cfg):
        self.cfg = cfg

        self.num_suggestions = self.cfg.suggester.num_suggestions
        self.remove_outliers = cfg.suggester.remove_outliers
        self.adjust_height = cfg.suggester.adjust_height

        if self.cfg.extrinsics_file:
            self.T_cam_to_world = np.load(self.cfg.extrinsics_file)["T"]
        else:
            self.T_cam_to_world = np.eye(4)
        self.T_world_to_cam = invert_transformation(self.T_cam_to_world)

        # Load model hyperparameters:
        with hydra.initialize(
            version_base=None, config_path=self.cfg.suggester.config_folder
        ):
            self.model_cfg = hydra.compose(self.cfg.suggester.config_file)

        # Load the model
        self.model = load_model(
            self.cfg.suggester.weights,
            has_pzX=True,
            conditioning=self.model_cfg.conditioning,
            cfg=self.model_cfg,
        )

    def get_data(self, pcd, pcd_seg, action_obj):
        # Get the action and anchor indexes
        action_idx = np.where(pcd_seg == action_obj)[
            0
        ]  # This is the target object's point cloud
        anchor_idx = np.where(pcd_seg != action_obj)[
            0
        ]  # The remaining environment is the anchor

        # Downsample to the min between the number of points and 4*1024
        action_idx = np.random.choice(action_idx, min(len(action_idx), 4 * 1024))
        anchor_idx = np.random.choice(anchor_idx, min(len(anchor_idx), 4 * 1024))

        action_pcd, action_mask = pcd[action_idx], pcd_seg[action_idx]
        anchor_pcd, anchor_mask = pcd[anchor_idx], pcd_seg[anchor_idx]
        data = {
            "clouds": np.concatenate([action_pcd, anchor_pcd]),
            "classes": np.concatenate(
                [np.zeros(len(action_pcd)), np.ones(len(anchor_pcd))]
            ),
            "masks": np.concatenate([action_mask, anchor_mask]),
        }

        return DataLoader(
            TestPointCloudDataset(
                data,
                action_class=0,
                anchor_class=1,
                cloud_type=self.model_cfg.cloud_type,
                pzY_input_dims=self.model_cfg.pzY_input_dims,
                num_points=self.model_cfg.num_points,
                rotation_variance=self.model_cfg.rotation_variance,
                translation_variance=self.model_cfg.translation_variance,
                downsample_type=self.model_cfg.conval_downsample_type,
                action_rot_sample_method=self.model_cfg.action_rot_sample_method,
                anchor_rot_sample_method=self.model_cfg.anchor_rot_sample_method,
                num_suggestions=self.num_suggestions,
                dataset_size=1,  # FIXME: This should be parallelized as num_objects
                seed=self.cfg.seed,  # FIXME: In the original evaluate_planning, this is always zero. This should be a hyperparameter
            ),
            batch_size=1,
            shuffle=False,
        )

    def suggest(self, pcd: np.ndarray, pcd_seg: np.ndarray, action_obj: int) -> list:
        """
        Input:
        pcd => point cloud, np.ndarray of size (n, 3)
        pcd_seg => segmentation ids , np.ndarray of size (n,)
        action_obj => object id to be moved, int

        Output:
        suggested_transforms => list of np.ndarrays of shape (4,4)
        """

        dataloader = self.get_data(
            pcd, pcd_seg, action_obj
        )  # TODO: We do not need a dataloader for inference, just make one function
        data = next(iter(dataloader))

        action_pcd = data["points_action_trans"]
        anchor_pcd = data["points_anchor_trans"]

        preds = self.model.get_transform(
            action_pcd,  # Downsampled point cloud of shape (1, 1024, 3) obtained from DataLoader
            anchor_pcd,  # Downsampled point cloud of shape (1, 1024, 3) obtained from DataLoader
            n_samples=self.num_suggestions,
            sampling_method=self.model_cfg.sampling_method,
        )

        probs, dists = self.get_probabilities(preds)

        # preds => list of 3 suggestions. Each one has keys ['pred_T_action', 'pred_points_action', 'flow_components']

        mean_point = data["mean_point"].squeeze(0).numpy()
        T0 = data["T0"].cpu().squeeze(0).detach().numpy()
        T1 = data["T1"].cpu().squeeze(0).detach().numpy()

        # undo the normalization performed prior evaluation
        Normalize = np.eye(4)
        Normalize[:3, 3] = -mean_point
        Normalize_inv = np.eye(4)
        Normalize_inv[:3, 3] = mean_point

        # Transform to the original frame
        T1_inv = T1.T
        T1_inv[:3, :3] = T1_inv[:3, :3].T
        T1_inv[:3, 3] = -T1_inv[:3, :3] @ T1_inv[:3, 3]

        suggested_transforms = []

        for pred in preds:
            T = pred["pred_T_action"].cpu().get_matrix().squeeze(0).detach().numpy()
            T = T1_inv @ T.T @ T0.T

            # Normalize the transformation
            T = Normalize_inv @ T @ Normalize

            full_action_pcd = pcd[pcd_seg == action_obj]

            if self.remove_outliers:
                full_action_pcd, _ = remove_outliers(
                    full_action_pcd,
                    inlier_ratio=self.cfg.collision.inlier_ratio,
                    radius=self.cfg.collision.radius,
                )

            # Filter the transformation with a rotation threshold:
            T_world = self.T_cam_to_world @ T @ self.T_world_to_cam
            full_action_pcd_world = transform_pcd(full_action_pcd, self.T_cam_to_world)
            T_world = filter_rotation(
                T_world,
                full_action_pcd_world,
                x_thresh=self.cfg.suggester.x_thresh,
                y_thresh=self.cfg.suggester.y_thresh,
                z_thresh=self.cfg.suggester.z_thresh,
            )
            T = self.T_world_to_cam @ T_world @ self.T_cam_to_world

            # transform the child node point cloud to the new frame
            child_node_pcd = transform_pcd(full_action_pcd, T)
            child_node_action_pcd_world = transform_pcd(
                child_node_pcd, self.T_cam_to_world
            )

            # plot_pcd(transform_pcd(child_node_pcd, self.T_cam_to_world), frame=True)

            # Transform to world frame for filtering:
            T_world = self.T_cam_to_world @ T @ self.T_world_to_cam

            if self.adjust_height:
                T_world = filter_translation(
                    T_world,
                    child_node_action_pcd_world,
                    min_height=self.cfg.suggester.min_height,
                    max_height=self.cfg.suggester.max_height,
                    min_movement=self.cfg.suggester.min_movement,
                )

            # Transform back to camera frame:
            T = self.T_world_to_cam @ T_world @ self.T_cam_to_world

            # plot_pcd(transform_pcd(child_node_pcd, self.T_cam_to_world), frame=True)

            suggested_transforms.append(T)
            if len(suggested_transforms) == self.num_suggestions:
                break

        return suggested_transforms, probs, dists

    def get_probabilities(self, preds):
        probs = []
        dists = []

        for i in range(len(preds)):
            action_distribution = preds[i]["flow_components"]["action_distribution"][0]
            anchor_distribution = preds[i]["flow_components"]["anchor_distribution"][0]
            action_sample = (
                preds[i]["flow_components"]["trans_sample_action"][0]
                .cpu()
                .detach()
                .numpy()
            )
            anchor_sample = (
                preds[i]["flow_components"]["trans_sample_anchor"][0]
                .cpu()
                .detach()
                .numpy()
            )

            goal_emb_cond_x = (
                preds[i]["flow_components"]["goal_emb_cond_x"][0, 0]
                .cpu()
                .detach()
                .numpy()
            )
            dists.append(goal_emb_cond_x)

            action_distribution = (
                softmax(action_distribution, dim=0).cpu().detach().numpy()
            )
            anchor_distribution = (
                softmax(anchor_distribution, dim=0).cpu().detach().numpy()
            )

            action_prob = action_distribution[action_sample == 1][0]
            anchor_prob = anchor_distribution[anchor_sample == 1][0]

            sample_prob = action_prob * anchor_prob
            probs.append(sample_prob)

        probs = np.array(probs)
        probs = probs / np.sum(probs)

        return probs, dists
