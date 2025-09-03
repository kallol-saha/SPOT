from copy import deepcopy
from typing import List, Tuple

import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
import pybullet as p
import torch
from pytorch3d.ops import sample_farthest_points
from scipy.spatial.transform import Rotation as R

COLORS = {
    "blue": np.array([78, 121, 167]) / 255.0,  # blue
    "green": np.array([89, 161, 79]) / 255.0,  # green
    "brown": np.array([156, 117, 95]) / 255.0,  # brown
    "orange": np.array([242, 142, 43]) / 255.0,  # orange
    "yellow": np.array([237, 201, 72]) / 255.0,  # yellow
    "gray": np.array([186, 176, 172]) / 255.0,  # gray
    "red": np.array([255, 87, 89]) / 255.0,  # red
    "purple": np.array([176, 122, 161]) / 255.0,  # purple
    "cyan": np.array([118, 183, 178]) / 255.0,  # cyan
    "pink": np.array([255, 157, 167]) / 255.0,
    "prediction": np.array([153, 255, 51]) / 255.0,
    "action": np.array([0, 128, 255]) / 255.0,
    "yellow cup": np.array([247, 155, 55]) / 255.0,
    "blue bowl": np.array([69, 105, 143]) / 255.0,
    "pink plate": np.array([255, 189, 172]) / 255.0,
    "green plate": np.array([167, 196, 160]) / 255.0,
    "blue plate": np.array([136, 206, 232]) / 255.0,
    "red bowl": np.array([211, 55, 68]) / 255.0,
    "yellow bowl": np.array([251, 222, 149]) / 255.0,
    "white cup": np.array([192, 170, 157]) / 255.0,
}


def get_color_mapping(pcd, pcd_seg, obj_id):
    colors = np.zeros((len(pcd), 3))

    # Set base color for mask value 0 (e.g., gray)
    colors[pcd_seg != obj_id] = [0.7, 0.7, 0.7]  # RGB values
    colormap = plt.cm.viridis

    # Get normalized z-coordinates for points with mask value 1
    mask_1_indices = np.where(pcd_seg == obj_id)[0]
    z_coords = pcd[mask_1_indices, 2]
    z_normalized = (z_coords - z_coords.min()) / (z_coords.max() - z_coords.min())

    # Apply colormap to masked points
    colors[mask_1_indices] = colormap(z_normalized)[:, :3]
    return colors


def plot_pcd(pcd, pcd_seg=None, frame=False, colormap="tab10"):
    if type(pcd) == torch.Tensor:
        pcd = pcd.cpu().detach().numpy()
    if pcd_seg is not None and type(pcd_seg) == torch.Tensor:
        pcd_seg = pcd_seg.cpu().detach().numpy()

    pts_vis = o3d.geometry.PointCloud()
    pts_vis.points = o3d.utility.Vector3dVector(pcd)

    if pcd_seg is not None:
        seg_ids = np.unique(pcd_seg)
        n = len(seg_ids)
        cmap = plt.get_cmap(colormap)
        id_to_color = {uid: cmap(i / n)[:3] for i, uid in enumerate(seg_ids)}

        # # for blocks
        # id_to_color = {
        #     0: COLORS["gray"],
        #     1: COLORS["brown"],
        #     2: COLORS["red"],
        #     3: COLORS["green"],
        #     4: COLORS["blue"],
        # }

        colors = np.array([id_to_color[seg_id] for seg_id in pcd_seg])
        pts_vis.colors = o3d.utility.Vector3dVector(colors)

    geometries = [pts_vis]

    if frame:
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.2, origin=[0, 0, 0]
        )
        geometries.append(frame)

    o3d.visualization.draw_geometries(geometries)


def plot_voxel_grid(voxel_grid, voxel_size):
    occupied_indices = np.array(np.where(voxel_grid == 1)).T
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(occupied_indices * voxel_size)
    voxel_grid_obj = o3d.geometry.VoxelGrid.create_from_point_cloud(
        pcd, voxel_size=np.max(voxel_size)
    )

    o3d.visualization.draw_geometries([voxel_grid_obj])


def plot_latent_space(points, latent_emb, sample_idx, distances):
    pcd = points[0].permute(1, 0).cpu().detach().numpy()
    index = sample_idx[0].cpu().detach().numpy()
    dist = distances[0].cpu().detach().numpy()
    prob = latent_emb[0].cpu().detach().numpy()
    prob = np.exp(prob)

    # seg_ids = np.unique(pcd_seg)
    # n = len(seg_ids)
    # cmap = plt.get_cmap("tab10")
    # id_to_color = {uid: cmap(i / n)[:3] for i, uid in enumerate(seg_ids)}

    # colors = np.array([id_to_color[seg_id] for seg_id in pcd_seg])
    # print("Seg IDs = ", seg_ids)
    # print("Colors = ", id_to_color)

    pts_vis = o3d.geometry.PointCloud()
    pts_vis.points = o3d.utility.Vector3dVector(pcd)

    # Visualize distribution mask:
    cmap = plt.get_cmap("plasma")
    max_dist = np.max(prob)
    min_dist = np.min(prob)
    colors = cmap((prob - min_dist) / (max_dist - min_dist))[:, :3]
    pts_vis.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([pts_vis])

    # Visualize discrete mask:
    colors = np.zeros((pcd.shape[0], 3))
    colors[index] = COLORS["red"]
    pts_vis.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([pts_vis])

    # Visualize distances mask:
    cmap = plt.get_cmap("viridis")
    max_dist = np.max(dist)
    min_dist = np.min(dist)
    colors = cmap((dist - min_dist) / (max_dist - min_dist))[:, :3]
    pts_vis.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([pts_vis])


def get_pointcloud(depth, width, height, view_matrix, proj_matrix, seg_img=None):
    """Returns a point cloud and its segmentation from the given depth image

    Args:
    -----
        depth (np.array): depth image
        width (int): width of the image
        height (int): height of the image
        view_matrix (np.array): 4x4 view matrix
        proj_matrix (np.array): 4x4 projection matrix
        seg_img (np.array): segmentation image

    Return:
    -------
        pcd (np.array): Nx3 point cloud
        pcd_seg (np.array): N array, segmentation of the point cloud
    """
    # based on https://stackoverflow.com/questions/59128880/getting-world-coordinates-from-opengl-depth-buffer

    # create a 4x4 transform matrix that goes from pixel coordinates (and depth values) to world coordinates
    proj_matrix = np.asarray(proj_matrix).reshape([4, 4], order="F")
    view_matrix = np.asarray(view_matrix).reshape([4, 4], order="F")
    tran_pix_world = np.linalg.inv(np.matmul(proj_matrix, view_matrix))

    # create a grid with pixel coordinates and depth values
    y, x = np.mgrid[-1 : 1 : 2 / height, -1 : 1 : 2 / width]
    y *= -1.0
    x, y, z = x.reshape(-1), y.reshape(-1), depth.reshape(-1)
    h = np.ones_like(z)

    pixels = np.stack([x, y, z, h], axis=1)
    # filter out "infinite" depths
    pixels = pixels[z < 0.99]
    pixels[:, 2] = 2 * pixels[:, 2] - 1

    if seg_img is not None:
        seg_img = np.array(seg_img)
        pcd_seg = seg_img.reshape(-1)[z < 0.99]
    else:
        pcd_seg = None

    # turn pixels to world coordinates
    points = np.matmul(tran_pix_world, pixels.T).T
    points /= points[:, 3:4]
    points = points[:, :3]

    return points, pcd_seg


def downsample_pcd(pcd, pcd_seg=None, num_points=1024):
    pcd_tensor = torch.tensor(pcd).unsqueeze(0)
    sampled_pcd_tensor, idx_tensor = sample_farthest_points(
        pcd_tensor, K=num_points, random_start_point=True
    )
    sampled_pcd_tensor = sampled_pcd_tensor.squeeze(0)
    idx = idx_tensor.squeeze(0).cpu().detach().numpy()

    sampled_pcd = sampled_pcd_tensor.cpu().detach().numpy()
    sampled_pcd_seg = pcd_seg[idx]

    return sampled_pcd, sampled_pcd_seg


# def downsample_full_pcd(cfg, pcd, pcd_seg, num_points):

#     obj_ids = [int(i) for i in cfg.planner.object_ids.split(",")]
#     obj_ids = [-1] + obj_ids        # For environment

#     actions = {}
#     anchors = {}

#     for i in range(len(obj_ids)):

#         action_pcd = pcd[pcd_seg == obj_ids[i]]
#         anchor_pcd = pcd[pcd_seg != obj_ids[i]]
#         action_pcd_seg = pcd_seg[pcd_seg == obj_ids[i]]
#         anchor_pcd_seg = pcd_seg[pcd_seg != obj_ids[i]]

#         downsampled_action, downsampled_action_seg = downsample_pcd(action_pcd, action_pcd_seg, cfg.num_points)
#         downsampled_anchor, downsampled_anchor_seg = downsample_pcd(anchor_pcd, anchor_pcd_seg, cfg.num_points)

#     sampled_pcd = sampled_pcd_tensor.cpu().detach().numpy()
#     sampled_pcd_seg = pcd_seg[idx]

#     return sampled_pcd, sampled_pcd_seg


def remove_outliers(
    point_cloud: np.ndarray, inlier_ratio: float = 0.372, radius: float = 0.12
):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud)

    cl, ind = pcd.remove_radius_outlier(
        nb_points=int(inlier_ratio * point_cloud.shape[0]), radius=radius
    )

    mask = np.zeros(point_cloud.shape[0], dtype=bool)
    mask[ind] = True

    return point_cloud[ind], mask

    # inlier_cloud = pcd.select_by_index(ind)
    # outlier_cloud = pcd.select_by_index(ind, invert=True)

    # # print("Showing outliers (red) and inliers (gray): ")
    # outlier_cloud.paint_uniform_color([1, 0, 0])
    # inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    # o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

    # print(max_z, min_z)


def remove_outliers_from_full_pcd(
    cfg,
    point_cloud: np.ndarray,
    seg: np.ndarray,
    inlier_ratio: float = 0.372,
    radius: float = 0.12,
):
    obj_ids = [int(i) for i in cfg.planner.object_ids.split(",")]
    full_mask = np.ones_like(seg, dtype=bool)

    for i in range(len(obj_ids)):
        obj_mask = seg == obj_ids[i]
        action_pcd = point_cloud[obj_mask]
        _, mask = remove_outliers(action_pcd, inlier_ratio=inlier_ratio, radius=radius)
        # full_mask[obj_mask] = (mask - 1) + (mask * seg[obj_mask])     # Where the mask is 0, replace with -1, making it part of the environment.
        full_mask[obj_mask] = mask

    return point_cloud[full_mask], seg[full_mask]


def remove_outliers_from_full_pcd_table_bussing(
    obj_ids: List[int],
    point_cloud: np.ndarray,
    seg: np.ndarray,
    inlier_ratio: float = 0.372,
    radius: float = 0.12,
):
    full_mask = np.ones_like(seg, dtype=bool)

    for i in range(len(obj_ids)):
        obj_mask = seg == obj_ids[i]
        action_pcd = point_cloud[obj_mask]
        _, mask = remove_outliers(action_pcd, inlier_ratio=inlier_ratio, radius=radius)
        # full_mask[obj_mask] = (mask - 1) + (mask * seg[obj_mask])     # Where the mask is 0, replace with -1, making it part of the environment.
        full_mask[obj_mask] = mask

    return point_cloud[full_mask], seg[full_mask]


def clean_outliers(pcd: np.ndarray, seg: np.ndarray, threshold: float = 0.1):
    """Remove the outliers from the point cloud based on the segmentation"""
    # Get the unique segments
    new_pcd, new_seg = [], []
    unique_segs = np.unique(seg)
    for seg_id in unique_segs:
        # Get the points of the segment
        idx = np.where(seg == seg_id)[0]
        pcd_seg = pcd[idx]

        # Use open3d to remove the outliers
        pcd_seg = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pcd_seg))
        # pcd_seg, valid_idx = pcd_seg.remove_radius_outlier(nb_points=8, radius=0.1)
        pcd_seg, valid_idx = pcd_seg.remove_statistical_outlier(
            nb_neighbors=20, std_ratio=5.0
        )

        # Update the point cloud
        new_pcd.append(np.asarray(pcd_seg.points))
        new_seg.append(np.ones(len(valid_idx)) * seg_id)

    new_pcd = np.concatenate(new_pcd)
    new_seg = np.concatenate(new_seg)

    return new_pcd, new_seg


def transform_pcd(pcd: np.ndarray, transform: np.ndarray) -> np.ndarray:
    """Transforms the given point cloud by the given transformation matrix.

    Args:
    -----
        pcd (np.ndarray): Nx3 point cloud
        transform (np.ndarray): 4x4 transformation matrix

    Returns:
    --------
            pcd_new (np.ndarray): Nx3 transformed point cloud
    """

    if pcd.shape[1] != 4:
        pcd = np.concatenate((pcd, np.ones((pcd.shape[0], 1))), axis=1)
    pcd_new = np.matmul(transform, pcd.T)[:-1, :].T
    return pcd_new


def homogenize_pcd_tensor(points: torch.Tensor):
    # points is of shape (B, N, 3)
    ones = torch.ones(*points.shape[:-1], 1, device=points.device)
    return torch.cat([points, ones], dim=-1)


def transform_pcd_tensor(points: torch.Tensor, T: torch.Tensor):
    homogenized = homogenize_pcd_tensor(points)
    transformed = torch.bmm(T, homogenized.transpose(-2, -1)).transpose(-2, -1)
    return transformed[..., :-1]


def transform_object_pcd(
    pcd: np.ndarray, pcd_seg: np.ndarray, transform: np.ndarray, obj: int
):
    new_pcd = pcd.copy()
    new_pcd[pcd_seg == obj] = transform_pcd(new_pcd[pcd_seg == obj], transform)

    return new_pcd


def apply_transform(
    pcd: np.ndarray,
    pcd_seg: np.ndarray,
    obj_id: str,
    transform: np.ndarray,
    plot: bool = False,
):
    """
    Transform the object in the point cloud by the specified transform.

    Importantly, preserves the order of the points in pcd.
    """
    if plot:
        colors = get_color_mapping(pcd, pcd_seg, obj_id)
        plot_pcd(pcd, pcd_seg, colors=colors)

    obj_pcd = pcd[pcd_seg == obj_id]
    transformed_pcd = transform_pcd(obj_pcd, transform)

    # Do not modify existing data
    new_pcd = deepcopy(pcd)
    new_pcd_seg = deepcopy(pcd_seg)

    # Modify in place to preserve the order of the points!
    new_pcd[new_pcd_seg == obj_id] = transformed_pcd

    if plot:
        plot_pcd(new_pcd, new_pcd_seg, colors=colors)

    return new_pcd, new_pcd_seg


def get_se3_transform(state1: np.ndarray, state2: np.ndarray):
    """Get the transformation matrix to go from the state1 to the state2"""
    # The rotation occurs in the object frame
    T = np.eye(4)
    T[:3, :3] = (R.from_quat(state2[3:]) * R.from_quat(state1[3:]).inv()).as_matrix()
    T_i = np.eye(4)
    T_i[:3, 3] = -np.array(state1[:3])
    T_f = np.eye(4)
    T_f[:3, 3] = state2[:3]

    return T_f @ T @ T_i


def rotation_matrix_to_euler_zyx(R):
    """
    Converts a rotation matrix to Euler angles (ZYX convention).

    Parameters:
        R (numpy.ndarray): 3x3 rotation matrix.

    Returns:
        tuple: (roll, pitch, yaw) in radians.
    """
    assert R.shape == (3, 3), "Input must be a 3x3 matrix"

    # Extract angles
    pitch = np.arcsin(-R[2, 0])  # R[2,0] = -sin(pitch)
    if np.abs(R[2, 0]) < 1:  # Check for gimbal lock
        yaw = np.arctan2(R[1, 0], R[0, 0])  # yaw = atan2(R[1,0], R[0,0])
        roll = np.arctan2(R[2, 1], R[2, 2])  # roll = atan2(R[2,1], R[2,2])
    else:  # Gimbal lock
        yaw = 0
        roll = np.arctan2(-R[0, 1], R[1, 1])  # roll depends on yaw

    return roll, pitch, yaw


# Example rotation matrix (3x3 submatrix of T1)
R = np.array([[0.5, -0.866, 0], [0.866, 0.5, 0], [0, 0, 1]])


def get_joint_transform(
    state1: float,
    state2: float,
    rot_axis: np.ndarray,
    joint_pos: np.ndarray = np.array([0, 0, 0]),
):
    """Get the transformation matrix generated by the joint rotation
    from state1 to state2 about the given axis."""
    # Get the transformation to change Reference frame
    Tf = np.eye(4)
    Tf[:3, 3] = -joint_pos[:3]

    # Get the inverse transformation to change Reference frame
    T_i = np.eye(4)
    T_i[:3, 3] = joint_pos[:3]

    # Get the rotation matrix
    T = np.eye(4)
    euler = rot_axis * (state2 - state1)
    quat = p.getQuaternionFromEuler(euler)
    T[:3, :3] = np.array(p.getMatrixFromQuaternion(quat)).reshape(3, 3)

    return T_i @ T @ Tf


def pad_pcds(
    pcds: List[np.ndarray], constant: int = 0
) -> Tuple[torch.tensor, torch.tensor]:
    """
    Pad a batch of point clouds.

    Returns the padded pointclouds as a single tensor, along with the number of points
    in each pointcloud.

    Args:
        pcds: length N list of arrays, pcds[i] has shape (P_i, D) or (P_i,)
    Returns:
        padded_pcds: (N, P, D) tensor, where P = max_i P_i
        lengths: (N,) tensor
    """
    max_len = max(pcd.shape[0] for pcd in pcds)

    padded_pcds = []
    lengths = []
    for pcd in pcds:
        lengths.append(pcd.shape[0])
        pad_len = max_len - pcd.shape[0]

        if len(pcd.shape) == 2:
            pad_width = ((0, pad_len), (0, 0))
        else:  # Input only has 1 dim
            pad_width = (0, pad_len)

        padded_pcd = np.pad(pcd, pad_width, constant_values=constant)
        padded_pcds.append(padded_pcd)

    padded_pcds = torch.from_numpy(np.stack(padded_pcds)).float()
    lengths = torch.tensor(lengths)
    assert padded_pcds.shape[0] == lengths.shape[0]

    return padded_pcds, lengths


def downsample_pcds(
    pcd_list: List[np.ndarray],
    other_data_lists: List[List[np.ndarray]],
    num_points: int = 1024,
):
    """
    Downsamples a batch of point clouds and other related data while maintaining
    point correspondences.

    Args:
        pcd_list: contains the batch of point clouds
        other_data_lists: variable length
    """
    # Pad the data to prepare for downsampling
    pcds, lengths = pad_pcds(pcd_list)

    padded_data: List[torch.tensor] = []
    for data in other_data_lists:
        data_tensor, lens = pad_pcds(data)
        assert torch.all(lengths == lens)
        padded_data.append(data_tensor)

    # pcds: (N, num_points, 3), idxs: (N, num_points)
    print("Downsampling start pcds using fps...")
    sampled_pcds, idxs = sample_farthest_points(
        pcds,
        lengths=lengths,
        K=num_points,
        random_start_point=True,
    )
    print("Finished")

    # Use the returned idxs to downsample actions correspondingly
    batch_idx = torch.arange(sampled_pcds.shape[0])[:, None]  # Shape: (N, 1)
    # batch_idx will broadcast to (N, num_points])
    sampled_other_data = []
    for data in padded_data:
        sampled_other_data.append(data[batch_idx, idxs])

    return sampled_pcds, tuple(sampled_other_data)
