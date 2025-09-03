import numpy as np
import torch
from torch import nn
import pytorch_lightning as pl
from pytorch3d.transforms import Rotate, random_rotations
from torchvision.transforms import ToTensor
import matplotlib.cm as cm
import wandb
import plotly.graph_objects as go

from torch.nn import functional as F
from placement_suggester.training.point_cloud_training_module import PointCloudTrainingModule
from placement_suggester.utils.emb_losses import (
    compute_occlusion_infonce_loss, infonce_loss, dist2weight, mean_order, mean_geo_diff, compute_infonce_loss)
from matplotlib.backends.backend_agg import FigureCanvasAgg
import matplotlib.pyplot as plt
# from sklearn.manifold import TSNE


mse_criterion = nn.MSELoss(reduction='sum')
to_tensor = ToTensor()


class EquivariancePreTrainingModule(PointCloudTrainingModule):

    def __init__(self,
                 model=None,
                 lr=1e-3,
                 image_log_period=500,
                 l2_reg_weight=0.00,
                 normalize_features=True,
                 temperature=0.1,
                 con_weighting='dist',
                 ):
        super().__init__(model=model, lr=lr,
                         image_log_period=image_log_period,)
        self.model = model
        self.lr = lr
        self.image_log_period = image_log_period
        self.l2_reg_weight = l2_reg_weight
        self.normalize_features = normalize_features
        self.temperature = temperature
        self.con_weighting = con_weighting

    def similarity_geo_distance(self, similarity, points):
        test_idx = np.random.randint(similarity.shape[1])
        dist = (points[0] - points[0, test_idx].unsqueeze(0)).norm(dim=-1)
        fig = plt.figure(figsize=(10, 7.5))

        ax_sim = fig.add_subplot(111)

        ax_sim.scatter(
            dist.detach().cpu().numpy(),
            similarity[0, test_idx].detach().cpu().numpy(),
        )
        ax_sim.set_ylabel('Similarity')
        ax_sim.set_xlabel('Geometric Distance')

        canvas = FigureCanvasAgg(fig)
        canvas.draw()
        img = np.array(canvas.buffer_rgba())
        plt.close(fig)
        return img

    def module_step(self, batch, batch_idx):
        points = batch['point_cloud']  # B, num_points, 3
        
        # loss, log_values = compute_infonce_loss(
        #     model=self.model,
        #     points=points,
        #     normalize_features=self.normalize_features,
        #     weighting=self.con_weighting,
        #     l2_reg_weight=self.l2_reg_weight
        # )
        
        loss, log_values = compute_occlusion_infonce_loss(
            model=self.model,
            points=points,
            normalize_features=self.normalize_features,
            weighting=self.con_weighting,
            l2_reg_weight=self.l2_reg_weight
        )
        
        return loss, log_values

    def visualize_results(self, batch, batch_idx):
        points = batch['point_cloud']  # B, num_points, 3
        transforms = Rotate(random_rotations(
            len(points), device=points.device))

        points_centered = points - \
            points.mean(dim=1, keepdims=True)  # B, num_points, 3

        points_trans = transforms.transform_points(points)
        points_trans_centered = points_trans - \
            points_trans.mean(dim=1, keepdims=True)
        # B, num_points, 3

        phi = self.model(points_centered.transpose(-1, -2))
        phi_trans = self.model(points_trans_centered.transpose(-1, -2))
        if(self.normalize_features):
            phi = F.normalize(phi, dim=1)
            phi_trans = F.normalize(phi_trans, dim=1)

        similarity = (phi.transpose(-1, -2) @ phi_trans)

        color = phi[0,:3].T.detach().cpu().numpy()
        color_trans = phi_trans[0,:3].T.detach().cpu().numpy()

        if(not self.normalize_features):
            colors_all = np.concatenate([color, color_trans], axis=0)
            c_min = colors_all.min(axis=0)
            c_max = colors_all.max(axis=0)
            color = 255*(color - c_min) / (c_max - c_min)
            color_trans = 255*(color_trans - c_min) / (c_max - c_min)
        else:
            color = 255*(color + 1)/2.
            color_trans = 255*(color_trans + 1)/2.
        
        points_emb = np.concatenate([points[0].detach().cpu().numpy(), color], axis=-1)
        points_trans_emb = np.concatenate([points[0].detach().cpu().numpy(), color_trans], axis=-1)
        
        test_idx = 100
        color_dist = 255 * \
            cm.viridis(similarity[0, test_idx].detach().cpu().numpy())[:, :3]
        points_comp_disp = np.concatenate(
            [points[0].detach().cpu().numpy(), color_dist], axis=-1)
        points_comp_disp[test_idx, 3:] = [255, 0, 0]
        res_viz = {}

        res_viz['points_emb'] = wandb.Object3D(points_emb)
        res_viz['points_trans_emb'] = wandb.Object3D(points_trans_emb)
        res_viz['points_comp_disp'] = wandb.Object3D(points_comp_disp)
        res_viz['sim_geo_distance'] = self.similarity_geo_distance(
            similarity, points)

        return res_viz
