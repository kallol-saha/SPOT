import numpy as np
import torch
from torch import nn
import pytorch_lightning as pl
from pytorch3d.transforms import Transform3d, Translate, matrix_to_axis_angle
from torchvision.transforms import ToTensor
from pytorch3d.loss import chamfer_distance
from placement_suggester.training.point_cloud_training_module import PointCloudTrainingModule
from placement_suggester.utils.se3 import dualflow2pose, flow2pose, get_translation, get_degree_angle
from placement_suggester.utils.display_headless import scatter3d, quiver3d
from placement_suggester.utils.color_utils import get_color

import wandb

mse_criterion = nn.MSELoss(reduction='sum')
to_tensor = ToTensor()


class EquivarianceTestingModule(PointCloudTrainingModule):

    def __init__(self,
                 model=None,
                 refinement_model=None,
                 lr=1e-3,
                 image_log_period=500,
                 action_weight=1,
                 anchor_weight=1,
                 smoothness_weight=0.1,
                 rotation_weight=0,
                 chamfer_weight=10000,
                 point_loss_type=0,
                 return_flow_component=False,
                 weight_normalize='l1',
                 loop=3
                 ):
        super().__init__(model=model, lr=lr,
                         image_log_period=image_log_period,)
        self.adaptation_model = model
        self.refinement_model = refinement_model
        self.lr = lr
        self.image_log_period = image_log_period
        self.action_weight = action_weight
        self.anchor_weight = anchor_weight
        self.smoothness_weight = smoothness_weight
        self.rotation_weight = rotation_weight
        self.chamfer_weight = chamfer_weight
        self.display_action = True
        self.display_anchor = True
        self.weight_normalize = weight_normalize
        # 0 for mse loss, 1 for chamfer distance, 2 for mse loss + chamfer distance
        self.point_loss_type = point_loss_type
        self.return_flow_component = return_flow_component
        self.loop = loop

    def get_transform(self, points_trans_action, points_trans_anchor):
        x_action, x_anchor = self.model(
            points_trans_action, points_trans_anchor)
        ans_dict = self.predict(x_action=x_action, x_anchor=x_anchor,
                                points_trans_action=points_trans_action, points_trans_anchor=points_trans_anchor)
        pred_T_action = ans_dict["pred_T_action"]
        pred_points_action = ans_dict["pred_points_action"]
        return ans_dict

    def predict(self, x_action, x_anchor, points_trans_action, points_trans_anchor):
        pred_flow_action, pred_w_action = self.extract_flow_and_weight(
            x_action)
        pred_flow_anchor, pred_w_anchor = self.extract_flow_and_weight(
            x_anchor)

        pred_T_action = dualflow2pose(xyz_src=points_trans_action, xyz_tgt=points_trans_anchor,
                                      flow_src=pred_flow_action, flow_tgt=pred_flow_anchor,
                                      weights_src=pred_w_action, weights_tgt=pred_w_anchor,
                                      return_transform3d=True, normalization_scehme=self.weight_normalize)

        pred_points_action = pred_T_action.transform_points(
            points_trans_action)

        return {"pred_T_action": pred_T_action,
                "pred_points_action": pred_points_action}

    def compute_loss(self, x_action, x_anchor, batch, log_values={}, loss_prefix='', pred_points_action=None):
        if 'T0' in batch.keys():
            T0 = Transform3d(matrix=batch['T0'])
            T1 = Transform3d(matrix=batch['T1'])

        if pred_points_action == None:
            points_trans_action = batch['points_action_trans']
        else:
            points_trans_action = pred_points_action
        points_trans_anchor = batch['points_anchor_trans']

        pred_flow_action, pred_w_action = self.extract_flow_and_weight(
            x_action)
        pred_flow_anchor, pred_w_anchor = self.extract_flow_and_weight(
            x_anchor)

        pred_T_action = dualflow2pose(xyz_src=points_trans_action, xyz_tgt=points_trans_anchor,
                                      flow_src=pred_flow_action, flow_tgt=pred_flow_anchor,
                                      weights_src=pred_w_action, weights_tgt=pred_w_anchor,
                                      return_transform3d=True, normalization_scehme=self.weight_normalize)
        pred_R_max, pred_R_min, pred_R_mean = get_degree_angle(pred_T_action)
        pred_t_max, pred_t_min, pred_t_mean = get_translation(pred_T_action)
        induced_flow_action = (pred_T_action.transform_points(
            points_trans_action) - points_trans_action).detach()
        pred_points_action = pred_T_action.transform_points(
            points_trans_action)

        R0_max, R0_min, R0_mean = get_degree_angle(T0)
        R1_max, R1_min, R1_mean = get_degree_angle(T1)
        t0_max, t0_min, t0_mean = get_translation(T0)
        t1_max, t1_min, t1_mean = get_translation(T1)

        log_values[loss_prefix+'R0_mean'] = R0_mean
        log_values[loss_prefix+'R0_max'] = R0_max
        log_values[loss_prefix+'R0_min'] = R0_min
        log_values[loss_prefix+'R1_mean'] = R1_mean
        log_values[loss_prefix+'R1_max'] = R1_max
        log_values[loss_prefix+'R1_min'] = R1_min

        log_values[loss_prefix+'t0_mean'] = t0_mean
        log_values[loss_prefix+'t0_max'] = t0_max
        log_values[loss_prefix+'t0_min'] = t0_min
        log_values[loss_prefix+'t1_mean'] = t1_mean
        log_values[loss_prefix+'t1_max'] = t1_max
        log_values[loss_prefix+'t1_min'] = t1_min

        log_values[loss_prefix+'pred_R_max'] = pred_R_max
        log_values[loss_prefix+'pred_R_min'] = pred_R_min
        log_values[loss_prefix+'pred_R_mean'] = pred_R_mean

        log_values[loss_prefix+'pred_t_max'] = pred_t_max
        log_values[loss_prefix+'pred_t_min'] = pred_t_min
        log_values[loss_prefix+'pred_t_mean'] = pred_t_mean
        loss = 0

        return loss, log_values, pred_points_action

    def extract_flow_and_weight(self, x):
        # x: Batch, num_points, 4
        pred_flow = x[:, :, :3]
        if(x.shape[2] > 3):
            pred_w = torch.sigmoid(x[:, :, 3])
        else:
            pred_w = None
        return pred_flow, pred_w

    def module_step(self, batch, batch_idx):
        points_trans_action = batch['points_action_trans']
        points_trans_anchor = batch['points_anchor_trans']
        log_values = {}

        x_action, x_anchor = self.model(
            points_trans_action, points_trans_anchor)
        loss, log_values, pred_points_action = self.compute_loss(
            x_action.detach(), x_anchor.detach(), batch, log_values=log_values, loss_prefix='', pred_points_action=points_trans_action)

        for i in range(self.loop):
            x_action, x_anchor = self.refinement_model(
                pred_points_action, points_trans_anchor)
            loss, log_values, pred_points_action = self.compute_loss(
                x_action.detach(), x_anchor.detach(), batch, log_values=log_values, loss_prefix=str(i), pred_points_action=pred_points_action)
        return 0, log_values

    def visualize_results(self, batch, batch_idx):

        points_trans_action = batch['points_action_trans']
        points_trans_anchor = batch['points_anchor_trans']
        self.predicted_action_list = []
        res_images = {}

        x_action, x_anchor = self.model(
            points_trans_action, points_trans_anchor)
        ans_dict = self.predict(
            x_action=x_action, x_anchor=x_anchor, points_trans_action=points_trans_action, points_trans_anchor=points_trans_anchor)
        pred_points_action = ans_dict["pred_points_action"]
        first_attempt = get_color(
            tensor_list=[points_trans_action[0],
                         ans_dict["pred_points_action"][0],
                         points_trans_anchor[0]],
            color_list=['blue', 'green', 'red'])
        res_images['first_attempt'] = wandb.Object3D(
            first_attempt)

        x_action, x_anchor = self.model(
            ans_dict["pred_points_action"], points_trans_anchor)

        ans_dict_refinement = self.predict(x_action=x_action, x_anchor=x_anchor,
                                           points_trans_action=ans_dict["pred_points_action"], points_trans_anchor=points_trans_anchor)

        second_attempt = get_color(
            tensor_list=[ans_dict["pred_points_action"][0],
                         ans_dict_refinement["pred_points_action"][0],
                         points_trans_anchor[0]],
            color_list=['blue', 'green', 'red'])
        res_images['second_attempt'] = wandb.Object3D(
            second_attempt)
        pred_points_action = ans_dict["pred_points_action"]
        self.predicted_action_list.append(pred_points_action)

        transformed_input_points = get_color(tensor_list=[
            points_trans_action[0], points_trans_anchor[0]], color_list=['blue', 'red'])
        res_images['input_points'] = wandb.Object3D(
            transformed_input_points)

        demo_points_apply_action_transform = get_color(
            tensor_list=[self.predicted_action_list[-1][0], points_trans_anchor[0]], color_list=['blue', 'red'])
        res_images['demo_points_apply_action_transform'] = wandb.Object3D(
            demo_points_apply_action_transform)
        return res_images
