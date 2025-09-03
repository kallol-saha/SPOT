from placement_suggester.training.flow_equivariance_training_module_nocentering import EquivarianceTrainingModule
from placement_suggester.dataset.point_cloud_data_module import MultiviewDataModule
from placement_suggester.models.transformer_flow import ResidualFlow_DiffEmbTransformer
import os
import torch

import numpy as np
import pytorch_lightning as pl

from placement_suggester.dataset.point_cloud_data_module import MultiviewDataModule
import hydra
from placement_suggester.utils.callbacks import SaverCallbackModel, SaverCallbackEmbnnActionAnchor
from pytorch_lightning.loggers import WandbLogger
# chuerp conda env: pytorch3d_38


@hydra.main(config_path="../configs", config_name="train_mug_residual")
def main(cfg):
    pl.seed_everything(cfg.seed)
    logger = WandbLogger(project=cfg.experiment)
    logger.log_hyperparams(cfg)
    logger.log_hyperparams({'working_dir': os.getcwd()})
    # trainer = pl.Trainer(logger=logger,
    #                      gpus=1,
    #                      reload_dataloaders_every_n_epochs=1,
    #                      callbacks=[SaverCallbackModel(), SaverCallbackEmbnnActionAnchor()],
    #                      max_steps=cfg.max_steps)

    # dm = MultiviewDataModule(
    #     dataset_root=cfg.dataset_root,
    #     test_dataset_root=cfg.test_dataset_root,
    #     dataset_index=cfg.dataset_index,
    #     action_class=cfg.action_class,
    #     anchor_class=cfg.anchor_class,
    #     dataset_size=cfg.dataset_size,
    #     rotation_variance=np.pi/180 * cfg.rotation_variance,
    #     translation_variance=cfg.translation_variance,
    #     batch_size=cfg.batch_size,
    #     num_workers=cfg.num_workers,
    #     num_demo=cfg.num_demo,
    #     cloud_type=cfg.cloud_type,
    #     num_points=cfg.num_points,
    #     overfit=cfg.overfit,
    #     synthetic_occlusion=cfg.synthetic_occlusion,
    #     ball_radius=cfg.ball_radius
    # )

    trainer = pl.Trainer(logger=logger,
                         gpus=1,
                         reload_dataloaders_every_n_epochs=1,
                         callbacks=[SaverCallbackModel(save_freq=cfg.ckpt_save_freq)],#, SaverCallbackEmbnnActionAnchorMultimodal()],
                         max_steps=cfg.max_steps,
                         gradient_clip_val=cfg.gradient_clipping) # 0 is no clipping

    dm = MultiviewDataModule(
        dataset_root=cfg.dataset_root,
        test_dataset_root=cfg.test_dataset_root,
        dataset_index=cfg.dataset_index,
        action_class=cfg.action_class,
        anchor_class=cfg.anchor_class,
        dataset_size=cfg.dataset_size,
        rotation_variance=np.pi/180 * cfg.rotation_variance,
        translation_variance=cfg.translation_variance,
        batch_size=cfg.batch_size,
        num_workers=cfg.num_workers,
        cloud_type=cfg.cloud_type,
        num_points=cfg.num_points,
        overfit=cfg.overfit,
        num_overfit_transforms=cfg.num_overfit_transforms,
        seed_overfit_transforms=cfg.seed_overfit_transforms,
        set_Y_transform_to_identity=cfg.set_Y_transform_to_identity,
        set_Y_transform_to_overfit=cfg.set_Y_transform_to_overfit,
        num_demo=cfg.num_demo,
        synthetic_occlusion=cfg.synthetic_occlusion,
        ball_radius=cfg.ball_radius,
        ball_occlusion=cfg.ball_occlusion,
        plane_occlusion=cfg.plane_occlusion,
        plane_standoff=cfg.plane_standoff,
        distractor_anchor_aug=cfg.distractor_anchor_aug,
        demo_mod_k_range=[cfg.demo_mod_k_range_min, cfg.demo_mod_k_range_max],
        demo_mod_rot_var=cfg.demo_mod_rot_var * np.pi/180,
        demo_mod_trans_var=cfg.demo_mod_trans_var,
        multimodal_transform_base=cfg.multimodal_transform_base
    )

    dm.setup()

    network = ResidualFlow_DiffEmbTransformer(
        emb_dims=cfg.emb_dims,
        emb_nn=cfg.emb_nn,
        return_flow_component=cfg.return_flow_component,
        center_feature=cfg.center_feature,
        inital_sampling_ratio=cfg.inital_sampling_ratio,
        pred_weight=cfg.pred_weight)
            
    model = EquivarianceTrainingModule(
        network,
        lr=cfg.lr,
        image_log_period=cfg.image_logging_period,
        point_loss_type=cfg.point_loss_type,
        rotation_weight=cfg.rotation_weight,
        weight_normalize=cfg.weight_normalize,
        consistency_weight=cfg.consistency_weight,
        smoothness_weight=cfg.smoothness_weight,
        sigmoid_on=cfg.sigmoid_on,
        softmax_temperature=cfg.softmax_temperature)

    model.cuda()
    model.train()
    if(cfg.checkpoint_file is not None):
        print("loaded checkpoint from")
        print(cfg.checkpoint_file)
        model.load_state_dict(torch.load(cfg.checkpoint_file)['state_dict'])

    else:
        if cfg.checkpoint_file_action is not None:
            model.model.emb_nn_action.load_state_dict(
                torch.load(cfg.checkpoint_file_action)['embnn_state_dict'])
            print(
                '-----------------------Pretrained EmbNN Action Model Loaded!-----------------------')
        if cfg.checkpoint_file_anchor is not None:
            model.model.emb_nn_anchor.load_state_dict(
                torch.load(cfg.checkpoint_file_anchor)['embnn_state_dict'])
            print(
                '-----------------------Pretrained EmbNN Anchor Model Loaded!-----------------------')

    trainer.fit(model, dm)


if __name__ == "__main__":

    torch.autograd.set_detect_anomaly(True)
    torch.cuda.empty_cache()
    torch.multiprocessing.set_sharing_strategy('file_system')
    main()
