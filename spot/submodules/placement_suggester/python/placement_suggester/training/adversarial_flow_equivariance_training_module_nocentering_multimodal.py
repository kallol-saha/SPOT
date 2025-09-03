import numpy as np
import torch
from torch import nn
from typing import Dict, Tuple

import plotly
import plotly.graph_objects as go
import pytorch_lightning as pl
from pytorch3d.transforms import Transform3d, Translate, matrix_to_axis_angle, Rotate, random_rotations
from torchvision.transforms import ToTensor
from pytorch3d.loss import chamfer_distance

from placement_suggester.models.multimodal_transformer_flow import Transformer, DGCNNClassification, DGCNN
from placement_suggester.models.pointnet2 import PointNet2SSG, PointNet2MSG
from placement_suggester.models.pointnet2pyg import PN2DenseWrapper, PN2DenseParams, PN2EncoderWrapper, PN2EncoderParams
from placement_suggester.models.transformer_flow import PointNet
from placement_suggester.models.vn_dgcnn import VN_DGCNN, VNArgs

from placement_suggester.training.point_cloud_training_module import PointCloudTrainingModule, AdversarialPointCloudTrainingModule

from placement_suggester.utils.se3 import dualflow2pose, flow2pose, get_translation, get_degree_angle, dense_flow_loss, pure_translation_se3
from placement_suggester.utils.color_utils import get_color, color_gradient
from placement_suggester.utils.display_headless import scatter3d, quiver3d
from placement_suggester.utils.emb_losses import compute_infonce_loss
from placement_suggester.utils.error_metrics import get_2rack_errors, get_all_sample_errors
from placement_suggester.utils.loss_utils import js_div, js_div_mod, wasserstein_distance

import torch.nn.functional as F

import wandb

mse_criterion = nn.MSELoss(reduction='sum')
to_tensor = ToTensor()


class Discriminator(nn.Module):
    def __init__(self, encoder_type: str = 'dgcnn', input_dims: int = 3, emb_dims: int = 512, 
                 transformer_emb_dims: int = 512, mlp_hidden_dims: int = 512, last_sigmoid: bool = False):
        """
        Create a discriminator to distinguish between demo/successful placement predictions
        and predicted placement predictions.
        
        Args:
            encoder_type (str): Type of encoder to use.
            input_dims (int): Number of input dimensions
            emb_dims (int): Number of embedding dimensions
            transformer_emb_dims (int): Number of embedding dimensions in the transformer
            mlp_hidden_dims (int): Number of hidden dimensions in the MLP
        """
        
        super(Discriminator, self).__init__()
        self.input_dims = input_dims

        # Create initial encoders
        self.encoder_type = encoder_type       
        self.encoder_emb_dims = emb_dims
        if self.encoder_type == 'mlp':
            self.action_encoder = nn.Sequential(
                PointNet([self.input_dims, self.encoder_emb_dims // 4, self.encoder_emb_dims // 2]),
                nn.Conv1d(self.encoder_emb_dims // 2, self.encoder_emb_dims, kernel_size=1, bias=False),
            )
            self.anchor_encoder = nn.Sequential(
                PointNet([self.input_dims, self.encoder_emb_dims // 4, self.encoder_emb_dims // 2]),
                nn.Conv1d(self.encoder_emb_dims // 2, self.encoder_emb_dims, kernel_size=1, bias=False),
            )
        
        elif self.encoder_type == 'dgcnn':
            self.action_encoder = DGCNN(input_dims=self.input_dims, emb_dims=self.encoder_emb_dims, num_heads=1, last_relu=False)
            self.anchor_encoder = DGCNN(input_dims=self.input_dims, emb_dims=self.encoder_emb_dims, num_heads=1, last_relu=False)
        
        else:
            raise ValueError('Encoder type not recognized')
        
        # Create transformer
        self.transformer_emb_dims = transformer_emb_dims
        self.transformer = Transformer(emb_dims=self.transformer_emb_dims, return_attn=True, bidirectional=False)
        
        # Create final MLP
        self.mlp_hidden_dims = mlp_hidden_dims
        self.mlp = nn.Sequential(
            nn.Linear(self.transformer_emb_dims, self.mlp_hidden_dims),
            nn.ReLU(),
            nn.Linear(self.mlp_hidden_dims, 1)   
        )

        self.last_sigmoid = last_sigmoid
        
        
    def forward(self, points_action: torch.Tensor, points_anchor: torch.Tensor) -> torch.Tensor:
        """
        Forward pass of the discriminator.
        
        Args:
            points_action (torch.Tensor): Points of the action, [B, D, N]
            points_anchor (torch.Tensor): Points of the anchor, [B, D, M]
            
        Returns:
            torch.Tensor: Discriminator output, [B, 1]
        """
        
        # Center by the mean of the anchor
        anchor_center_xyz = points_anchor[:, :3, :].mean(dim=-1, keepdim=True)
        points_action_xyz_centered = points_action[:, :3, :] - anchor_center_xyz
        points_anchor_xyz_centered = points_anchor[:, :3, :] - anchor_center_xyz
        
        points_action_centered = torch.cat([points_action_xyz_centered, points_action[:, 3:, :]], dim=1)
        points_anchor_centered = torch.cat([points_anchor_xyz_centered, points_anchor[:, 3:, :]], dim=1)

        # Encode the action and anchor
        action_embedding = self.action_encoder(points_action_centered)
        anchor_embedding = self.anchor_encoder(points_anchor_centered)
        
        # Transformer
        placement_embedding, placement_attn = self.transformer(action_embedding, anchor_embedding)
        
        # Mean pooling
        placement_embedding = placement_embedding.mean(dim=-1)

        # MLP
        if self.last_sigmoid:
            out = F.sigmoid(self.mlp(placement_embedding))
        else:
            out = self.mlp(placement_embedding)
            
        return out, placement_attn
        

class AdversarialEquivarianceTrainingModule_WithPZCondX(AdversarialPointCloudTrainingModule):
    def __init__(self, model_cond_x, discriminator: Discriminator, 
                 lr: float = 1, image_log_period: int = 100, gradient_clipping: float = None,
                 generator_loss_weight: float = 1.0, discriminator_loss_weight: float = 1.0, 
                 freeze_taxpose: bool = False):
        """
        Wrapper for the adversarial training module for p(z|X).
        
        Args:
            model_cond_x (EquivarianceTrainingModule_WithPZCondX): p(z|X) training module
            discriminator (Discriminator): Discriminator
            lr (float): Learning rate
            image_log_period (int): Image log period
            gradient_clipping (float): Gradient clipping value
            generator_loss_weight (float): Weight of the generator loss
            discriminator_loss_weight (float): Weight of the discriminator loss
            freeze_taxpose (bool): Whether to freeze the taxpose model
        """
        
        super(AdversarialEquivarianceTrainingModule_WithPZCondX, self).__init__(generator=model_cond_x, 
                                                                                discriminator=discriminator,
                                                                                generator_loss_weight=generator_loss_weight,
                                                                                discriminator_loss_weight=discriminator_loss_weight,
                                                                                gradient_clipping=gradient_clipping,
                                                                                lr=lr,
                                                                                image_log_period=image_log_period,)

        self.model_cond_x = model_cond_x
        self.discriminator = discriminator
        self.discriminator_loss_weight = discriminator_loss_weight
        self.freeze_taxpose = freeze_taxpose
        
        if self.freeze_taxpose:
            print(f'Freezing taxpose model')
            for param in self.model_cond_x.model.tax_pose.parameters():
                param.requires_grad = False
                
    def maybe_freeze_parameters(self):
        """
        Freeze parameters if necessary.
        """
        
        if self.freeze_taxpose:
            for param in self.model_cond_x.model.tax_pose.parameters():
                param.requires_grad = False
                
    def maybe_unfreeze_parameters(self):
        """
        Unfreeze parameters if necessary.
        """
        
        if self.freeze_taxpose:
            for param in self.model_cond_x.model.tax_pose.parameters():
                param.requires_grad = True

    def adversarial_module_step(self, batch: Dict, batch_idx: int) -> Tuple[torch.Tensor, torch.Tensor, Dict]:
        """
        Adversarial module step for p(z|X). First gets a predicted transform using p(z|X), then runs the discriminator.    
        
        Args:
            batch (Dict): Batch of data
            batch_idx (int): Batch index
            
        Returns:
            discriminator_loss (torch.Tensor): Discriminator loss
            generator_loss (torch.Tensor): Generator loss
            log_values (Dict): Log values
        """
        
        points_action = batch['points_action']
        points_anchor = batch['points_anchor']
        points_trans_action = batch['points_action_trans']
        points_trans_anchor = batch['points_anchor_trans']
        points_onetrans_action = batch['points_action_onetrans'] if 'points_action_onetrans' in batch else batch['points_action']
        points_onetrans_anchor = batch['points_anchor_onetrans'] if 'points_anchor_onetrans' in batch else batch['points_anchor']

        T1 = Transform3d(matrix=batch['T1'])
        
        # Run p(z|X) forward pass
        pzX_loss, pzX_log_values, model_outputs = self.model_cond_x.module_step(batch, batch_idx)
        
        if self.model_cond_x.model.tax_pose.return_flow_component: 
            x_action = model_outputs[0]['flow_action']
            x_anchor = model_outputs[0]['flow_anchor']
            goal_emb = model_outputs[0]['goal_emb']
            goal_emb_cond_x = model_outputs[0]['goal_emb_cond_x']
        else:
            if self.model_cond_x.model_with_cond_x.conditioning not in ["latent_z", "latent_z_1pred", "latent_z_1pred_10d", "latent_z_linear", "latent_z_linear_internalcond"]:
                if self.model_with_cond_x.return_debug:
                    x_action, x_anchor, goal_emb, goal_emb_cond_x, for_debug = model_outputs[0]
                    heads = None
                else:
                    x_action, x_anchor, goal_emb, goal_emb_cond_x = model_outputs[0]
                    heads = None
            else:
                if self.model_cond_x.model_with_cond_x.return_debug:
                    x_action, x_anchor, goal_emb, goal_emb_cond_x, heads, for_debug = model_outputs[0]
                else:
                    x_action, x_anchor, goal_emb, goal_emb_cond_x, heads = model_outputs[0]

        # Make sure to follow RelDist sampling if it occurred
        if "sampled_ixs_action" in model_outputs[0]:
            ixs_action = model_outputs[0]["sampled_ixs_action"].unsqueeze(-1)
            sampled_points_action = torch.take_along_dim(
                points_action, ixs_action, dim=1
            )
            sampled_points_trans_action = torch.take_along_dim(
                points_trans_action, ixs_action, dim=1
            )
        else:
            sampled_points_action = points_action
            sampled_points_trans_action = points_trans_action

        if "sampled_ixs_anchor" in model_outputs[0]:
            ixs_anchor = model_outputs[0]["sampled_ixs_anchor"].unsqueeze(-1)
            sampled_points_anchor = torch.take_along_dim(
                points_anchor, ixs_anchor, dim=1
            )
            sampled_points_trans_anchor = torch.take_along_dim(
                points_trans_anchor, ixs_anchor, dim=1
            )
        else:
            sampled_points_anchor = points_anchor
            sampled_points_trans_anchor = points_trans_anchor

        # Extract the predicted flow and weights
        pred_flow_action, pred_w_action = self.extract_flow_and_weight(x_action)
        pred_flow_anchor, pred_w_anchor = self.extract_flow_and_weight(x_anchor)

        # Extract the predicted transform using weighted SVD
        pred_T_action = dualflow2pose(xyz_src=sampled_points_trans_action[:, :, :3], 
                                      xyz_tgt=sampled_points_trans_anchor[:, :, :3],
                                      flow_src=pred_flow_action, 
                                      flow_tgt=pred_flow_anchor,
                                      weights_src=pred_w_action, 
                                      weights_tgt=pred_w_anchor,
                                      return_transform3d=True, 
                                      normalization_scehme=self.model_cond_x.training_module_no_cond_x.weight_normalize, 
                                      temperature=self.model_cond_x.training_module_no_cond_x.softmax_temperature)
        
        # Apply the predicted transform and then transform back to the original action/anchor frame
        pred_points_action_xyz = pred_T_action.transform_points(points_trans_action[:, :, :3])
        pred_points_action_orig_frame_xyz = T1.inverse().transform_points(pred_points_action_xyz)
        pred_points_action_orig_frame = torch.cat([pred_points_action_orig_frame_xyz, points_action[:, :, 3:]], dim=-1)

        return pzX_loss, pzX_log_values, pred_points_action_orig_frame

    def action_centered(self, *args, **kwargs):
        return self.model_cond_x.action_centered(*args, **kwargs)
    
    def get_transform(self, *args, **kwargs):
        return self.model_cond_x.get_transform(*args, **kwargs)
    
    def predict(self, *args, **kwargs):
        return self.model_cond_x.predict(*args, **kwargs)    
    
    def compute_loss(self, *args, **kwargs):
        return self.model_cond_x.compute_loss(*args, **kwargs)
    
    def extract_flow_and_weight(self, *args, **kwargs):
        return self.model_cond_x.extract_flow_and_weight(*args, **kwargs)
    
    def module_step(self, *args, **kwargs):
        return self.model_cond_x.module_step(*args, **kwargs)
    
    def visualize_results(self, *args, **kwargs):
        return self.model_cond_x.visualize_results(*args, **kwargs)