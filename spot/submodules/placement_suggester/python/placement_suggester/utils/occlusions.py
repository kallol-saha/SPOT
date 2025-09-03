import torch
import torch.nn.functional as F
from pytorch3d.ops import ball_query

def ball_occlusion(points, radius = 0.05):
    N = points.shape[0]
    idx = torch.randint(N,[1])
    center = points[idx]
    ret = ball_query(center.unsqueeze(0), points.unsqueeze(0), radius=radius, K=N)
    mask = torch.isin(torch.arange(points.shape[0], device=points.device), ret.idx[0], invert=True)
    return points[mask]

def plane_occlusion(points, stand_off=0.02):
    idx = torch.randint(points.shape[0],[1])
    pt = points[idx]
    center = points.mean(dim=0, keepdim=True)
    plane_norm = F.normalize(pt-center, dim=-1)
    plane_orig = pt - stand_off*plane_norm
    points_vec = F.normalize(points-plane_orig, dim=-1)
    split = plane_norm @ points_vec.transpose(-1,-2)
    mask = split[0] < 0
    return points[mask]