from pytorch3d.ops import ball_query
import torch
from torch.nn import functional as F


def ball_occlusion(points, radius=0.05, return_mask=False):
    """
    points: (num_points, 3)
    """
    idx = torch.randint(points.shape[0], [1])
    center = points[idx]
    sampled_radius = (radius - 0.025) * torch.rand(1) + 0.025
    ret = ball_query(center.unsqueeze(0), points.unsqueeze(0), radius=sampled_radius, K=points.shape[0])
    mask = torch.isin(torch.arange(
        points.shape[0], device=points.device), ret.idx[0], invert=True)
    if return_mask:
        return points[mask], mask
    return points[mask]


def plane_occlusion(points, stand_off=0.02, return_mask=False):
    idx = torch.randint(points.shape[0], [1])
    pt = points[idx]
    center = points.mean(dim=0, keepdim=True)
    plane_norm = F.normalize(pt-center, dim=-1)
    plane_orig = pt - stand_off*plane_norm
    points_vec = F.normalize(points-plane_orig, dim=-1)
    split = plane_norm @ points_vec.transpose(-1, -2)
    mask = split[0] < 0
    if return_mask:
        return points[mask], mask
    return points[mask]


def bottom_surface_occlusion(points, z_clipping_height=0.01, return_mask=False):
    min_z = points[:, 2].min()
    mask = points[:, 2] > min_z + z_clipping_height
    if return_mask:
        return points[mask], mask
    return points[mask]