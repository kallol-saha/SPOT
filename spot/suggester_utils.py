import numpy as np
import torch
from scipy.spatial.transform import Rotation as R
from spot.utils.pcd_utils import transform_pcd


def set_seed(seed):
    # Set seed for NumPy
    np.random.seed(seed)

    # Set seed for PyTorch
    torch.manual_seed(seed)

    # If using CUDA, set seed for GPU operations as well
    if torch.cuda.is_available():
        torch.cuda.manual_seed(seed)
        torch.cuda.manual_seed_all(seed)  # If using multi-GPU

    # Enable deterministic behavior for reproducibility
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False


def invert_transformation(T):
    """
    Invert a 4x4 similarity transformation matrix.

    Args:
        T (np.ndarray): A 4x4 transformation matrix.

    Returns:
        np.ndarray: The inverted 4x4 transformation matrix.
    """
    assert T.shape == (4, 4), "Input must be a 4x4 matrix."

    # Extract rotation and scaling (upper-left 3x3) and translation (top-right 3x1)
    RS = T[:3, :3]
    t = T[:3, 3]

    # Invert rotation and scaling
    RS_inv = np.linalg.inv(RS)  # Assumes RS is non-singular

    # Invert translation
    t_inv = -RS_inv @ t

    # Construct the inverted transformation matrix
    T_inv = np.eye(4)
    T_inv[:3, :3] = RS_inv
    T_inv[:3, 3] = t_inv

    return T_inv


def placement_height_error(
    child_node_action_pcd, T_cam_to_world, min_height, max_height
):
    child_node_action_pcd_world = transform_pcd(child_node_action_pcd, T_cam_to_world)
    min_z = np.min(child_node_action_pcd_world[:, 2])
    max_z = np.max(child_node_action_pcd_world[:, 2])

    move_up_by = np.clip(min_height - min_z, 0, None)
    move_down_by = np.clip(max_height - max_z, None, 0)
    if move_up_by > 0:
        return move_up_by
    else:
        return move_down_by
    # obj_pcd[:, 2] = obj_pcd[:, 2] + move_up_by
    # child_node_pcd_world[initial_pcd_seg == obj] = obj_pcd
    # T_world_to_cam = invert_transformation(T_cam_to_world)
    # child_node_pcd = transform_pcd(child_node_pcd_world, T_world_to_cam)


def filter_translation(T, pcd_world, min_height, max_height, min_movement):
    R = T[:3, :3]  # Top-left 3x3 matrix
    t = T[:3, 3]  # Top-right 3x1 vector

    # Step 1: Convert transformed translation to original frame
    t_world = -R.T @ t
    # print(f'Original translation: {t_world}')

    # Filter placement height:
    min_z = np.min(pcd_world[:, 2])
    max_z = np.max(pcd_world[:, 2])
    # print(f"Min Z: {min_z}, Max Z: {max_z}")

    move_up_by = np.clip(min_height - min_z, 0, None)
    move_down_by = np.clip(max_height - max_z, None, 0)
    # print(f"Move up by: {move_up_by}, Move down by: {move_down_by}")
    if move_up_by > 0:
        height_error = move_up_by
    else:
        height_error = move_down_by

    # Add height error to the translation in the original frame
    t_world[2] = t_world[2] - height_error

    movement = np.linalg.norm(t_world[:2])  # Movement in the x-y plane
    if movement < min_movement:
        t_world[:2] = t_world[:2] * (min_movement / movement)

    # print(f"Filtered translation: {t_world}")

    # Convert back the translation:
    t_filtered = -R @ t_world
    T[:3, 3] = t_filtered

    return T.copy()


def filter_rotation(T, action_pcd, x_thresh=30.0, y_thresh=30.0, z_thresh=120.0):
    """
    Check if the rotation matrix exceeds specified thresholds for x, y, z rotations.

    Parameters:
    matrix (np.ndarray): 3x3 rotation matrix.
    x_thresh (float): Threshold for rotation about the x-axis (roll) in degrees.
    y_thresh (float): Threshold for rotation about the y-axis (pitch) in degrees.
    z_thresh (float): Threshold for rotation about the z-axis (yaw) in degrees.

    Returns:
    bool: True if all rotations are within thresholds, False otherwise.
    """
    # Validate the input is a proper rotation matrix
    # if not (matrix.shape == (3, 3) and np.allclose(np.dot(matrix.T, matrix), np.eye(3), atol=1e-6)):
    #     raise ValueError("Input must be a valid 3x3 rotation matrix.")

    # import pdb; pdb.set_trace()

    mean_point = np.mean(action_pcd, axis=0)
    T_camtomean = np.eye(4)
    T_camtomean[:3, 3] = -mean_point
    T_meantocam = invert_transformation(T_camtomean)

    # Calculate the transformation that would be applied to the mean centered point cloud
    T_mean = T @ T_meantocam

    # Get the rotation matrix from it:
    r_matrix = T_mean[:3, :3]

    # Extract Euler angles using scipy's Rotation class
    r = R.from_matrix(r_matrix)
    roll, pitch, yaw = r.as_euler("xyz", degrees=True)  # Roll, pitch, yaw in degrees

    # Clip the angles to the thresholds
    roll_clipped = np.clip(roll, -x_thresh, x_thresh)
    pitch_clipped = np.clip(pitch, -y_thresh, y_thresh)
    yaw_clipped = np.clip(yaw, -z_thresh, z_thresh)

    # Reconstruct the rotation matrix from the clipped angles
    clipped_rotation = R.from_euler(
        "xyz", [roll_clipped, pitch_clipped, yaw_clipped], degrees=True
    )
    r_matrix_clipped = clipped_rotation.as_matrix()

    # Replace the rotation part of the transformation matrix with the clipped rotation
    T_mean_clipped = T_mean.copy()
    T_mean_clipped[:3, :3] = r_matrix_clipped

    # Convert the clipped transformation back to the camera frame
    T_clipped = T_mean_clipped @ T_camtomean

    return T_clipped
