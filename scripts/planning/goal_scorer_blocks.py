import numpy as np

from spot.heuristics import _get_block_poses


def score_goal(pcd, pcd_seg):
    # Assumes block stacking
    poses = np.stack(_get_block_poses(pcd, pcd_seg))

    zs = [pose[2] for pose in poses]
    bottom = poses[np.argmin(zs)]

    xy_diffs = (poses - bottom)[:, :2]
    # Greater L2 distance is worse
    return -np.linalg.norm(xy_diffs, axis=1).sum()
