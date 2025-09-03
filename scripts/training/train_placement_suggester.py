# from placement_suggester.dataset.rpdiff_data_module import RpDiffDataModule
import os
import torch
import torch.multiprocessing
import torch.nn as nn
from placement_suggester.dataset.point_cloud_data_module import MultiviewDataModule
from placement_suggester.models.multimodal_transformer_flow import (
    Multimodal_ResidualFlow_DiffEmbTransformer,
    Multimodal_ResidualFlow_DiffEmbTransformer_WithPZCondX,
)
from placement_suggester.models.transformer_flow import (
    AlignedFrameDecoder,
    ResidualFlow_DiffEmbTransformer,
)
from placement_suggester.training.adversarial_flow_equivariance_training_module_nocentering_multimodal import (
    AdversarialEquivarianceTrainingModule_WithPZCondX,
    Discriminator,
)
from placement_suggester.training.flow_equivariance_training_module_nocentering_multimodal import (
    EquivarianceTrainingModule,
    EquivarianceTrainingModule_WithPZCondX,
)

torch.multiprocessing.set_sharing_strategy("file_system")

import hydra
import numpy as np
import pytorch_lightning as pl
from placement_suggester.dataset.point_cloud_data_module import MultiviewDataModule
from placement_suggester.utils.callbacks import SaverCallbackModel
from pytorch_lightning.strategies import DDPStrategy

import wandb
from spot.utils.path_utils import resolve_config_paths

def setup_main(cfg):
    # Resolve relative paths to absolute paths
    cfg = resolve_config_paths(cfg)
    
    wandb.init()

    pl.seed_everything(cfg.seed)
    # if cfg.resume_id is None:
    #     logger = WandbLogger(project=cfg.experiment, group=cfg.wandb_group)
    # else:
    #     logger = WandbLogger(
    #         project=cfg.experiment,
    #         group=cfg.wandb_group,
    #         id=cfg.resume_id,
    #         resume="must",
    #     )
    # logger.log_hyperparams(cfg)
    # logger.log_hyperparams({"working_dir": os.getcwd()})

    # Have to handle adversarial p(z|X) separately because of manual optimization
    gradient_clipping = cfg.gradient_clipping

    trainer = pl.Trainer(
        # logger=logger,  # !!! I am ignoring the logger for now because I don't immediately need logging
        #  gpus=1,
        reload_dataloaders_every_n_epochs=1,
        callbacks=[
            SaverCallbackModel(save_freq=cfg.ckpt_save_freq)
        ],  # , SaverCallbackEmbnnActionAnchorMultimodal()],
        max_steps=cfg.max_steps,
        gradient_clip_val=(
            cfg.gradient_clipping if not cfg.pzX_adversarial else None
        ),  # 0 is no clipping
        check_val_every_n_epoch=cfg.check_val_every_n_epoch,
        strategy=DDPStrategy(find_unused_parameters=True),
    )

    # if cfg.init_cond_x and cfg.distractor_anchor_aug:
    #     assert cfg.multimodal_transform_base, "Are you sure you don't want to rotate demo rack when training p(z|Y)?"
    if not cfg.init_cond_x and cfg.distractor_anchor_aug:  # This is False, don't bother
        assert (
            not cfg.multimodal_transform_base
        ), "Are you sure you want to rotate demo rack when training p(z|Y)?"

    ####################

    # dm = RpDiffDataModule(batch_size=cfg.batch_size, obj_config=cfg.dataset_root)
    # dm.setup()

    ##################

    dm = MultiviewDataModule(  # !!! This is the datamodule or data loader
        dataset_root=cfg.dataset_root,
        test_dataset_root=cfg.test_dataset_root,
        dataset_index=cfg.dataset_index,
        action_class=cfg.action_class,
        anchor_class=cfg.anchor_class,
        dataset_size=cfg.dataset_size,
        rotation_variance=np.pi / 180 * cfg.rotation_variance,
        translation_variance=cfg.translation_variance,
        batch_size=cfg.batch_size,
        num_workers=cfg.num_workers,
        cloud_type=cfg.cloud_type,
        num_points=cfg.num_points,
        overfit=cfg.overfit,
        overfit_distractor_aug=cfg.overfit_distractor_aug,
        num_overfit_transforms=cfg.num_overfit_transforms,
        seed_overfit_transforms=cfg.seed_overfit_transforms,
        set_Y_transform_to_identity=cfg.set_Y_transform_to_identity,
        set_Y_transform_to_overfit=cfg.set_Y_transform_to_overfit,
        num_demo=cfg.num_demo,
        synthetic_occlusion=cfg.synthetic_occlusion,
        ball_radius=cfg.ball_radius,
        plane_standoff=cfg.plane_standoff,
        bottom_surface_z_clipping_height=cfg.bottom_surface_z_clipping_height,
        scale_point_clouds=cfg.scale_point_clouds,
        scale_point_clouds_min=cfg.scale_point_clouds_min,
        scale_point_clouds_max=cfg.scale_point_clouds_max,
        distractor_anchor_aug=cfg.distractor_anchor_aug,
        demo_mod_k_range=[cfg.demo_mod_k_range_min, cfg.demo_mod_k_range_max],
        demo_mod_rot_var=cfg.demo_mod_rot_var * np.pi / 180,
        demo_mod_trans_var=cfg.demo_mod_trans_var,
        multimodal_transform_base=cfg.multimodal_transform_base,
        action_rot_sample_method=cfg.action_rot_sample_method,
        anchor_rot_sample_method=cfg.anchor_rot_sample_method,
        distractor_rot_sample_method=cfg.distractor_rot_sample_method,
        skip_failed_occlusion=cfg.skip_failed_occlusion,
        min_num_cameras=cfg.min_num_cameras,
        max_num_cameras=cfg.max_num_cameras,
        use_consistent_validation_set=cfg.use_consistent_validation_set,
        use_all_validation_sets=cfg.use_all_validation_sets,
        conval_rotation_variance=np.pi / 180 * cfg.conval_rotation_variance,
        conval_translation_variance=cfg.conval_translation_variance,
        conval_synthetic_occlusion=cfg.conval_synthetic_occlusion,
        conval_scale_point_clouds=cfg.conval_scale_point_clouds,
        conval_action_rot_sample_method=cfg.conval_action_rot_sample_method,
        conval_anchor_rot_sample_method=cfg.conval_anchor_rot_sample_method,
        conval_distractor_rot_sample_method=cfg.conval_distractor_rot_sample_method,
        conval_min_num_cameras=cfg.conval_min_num_cameras,
        conval_max_num_cameras=cfg.conval_max_num_cameras,
        conval_downsample_type=cfg.conval_downsample_type,
        conval_gaussian_noise_mu=cfg.conval_gaussian_noise_mu,
        conval_gaussian_noise_std=cfg.conval_gaussian_noise_std,
        use_class_labels=cfg.use_class_labels,
        action_occlusion_class=cfg.action_occlusion_class,
        action_plane_occlusion=cfg.action_plane_occlusion,
        action_ball_occlusion=cfg.action_ball_occlusion,
        action_bottom_surface_occlusion=cfg.action_bottom_surface_occlusion,
        anchor_occlusion_class=cfg.anchor_occlusion_class,
        anchor_plane_occlusion=cfg.anchor_plane_occlusion,
        anchor_ball_occlusion=cfg.anchor_ball_occlusion,
        anchor_bottom_surface_occlusion=cfg.anchor_bottom_surface_occlusion,
        downsample_type=cfg.downsample_type,
        gaussian_noise_mu=cfg.gaussian_noise_mu,
        gaussian_noise_std=cfg.gaussian_noise_std,
        return_rpdiff_mesh_files=cfg.compute_rpdiff_min_errors,
    )
    dm.setup()

    TP_input_dims = Multimodal_ResidualFlow_DiffEmbTransformer.TP_INPUT_DIMS[
        cfg.conditioning
    ]
    if cfg.conditioning in [
        "latent_z_linear",
        "hybrid_pos_delta_l2norm",
        "hybrid_pos_delta_l2norm_global",
    ]:
        TP_input_dims += (
            cfg.latent_z_linear_size
        )  # Hacky way to add the dynamic latent z to the input dims

    # if cfg.conditioning in ["latent_z_linear"]:
    #     assert not cfg.freeze_embnn and not cfg.freeze_z_embnn and not cfg.freeze_residual_flow, "Probably don't want to freeze the network when training the latent model"
    if cfg.decoder_type == "taxpose":  # This is satisfied in the config
        inner_network = ResidualFlow_DiffEmbTransformer(  # !!! This is the inner model
            emb_dims=cfg.emb_dims,
            input_dims=TP_input_dims,
            emb_nn=cfg.emb_nn,
            return_flow_component=cfg.return_flow_component,
            center_feature=cfg.center_feature,
            inital_sampling_ratio=cfg.inital_sampling_ratio,
            pred_weight=cfg.pred_weight,
            freeze_embnn=cfg.freeze_embnn,
            conditioning_size=(
                cfg.latent_z_linear_size
                if cfg.conditioning
                in [
                    "latent_z_linear_internalcond",
                    "hybrid_pos_delta_l2norm_internalcond",
                    "hybrid_pos_delta_l2norm_global_internalcond",
                ]
                else 0
            ),
            multilaterate=cfg.multilaterate,
            sample=cfg.mlat_sample,
            mlat_nkps=cfg.mlat_nkps,
            pred_mlat_weight=cfg.pred_mlat_weight,
            conditioning_type=cfg.taxpose_conditioning_type,
            flow_head_use_weighted_sum=cfg.flow_head_use_weighted_sum,
            flow_head_use_selected_point_feature=cfg.flow_head_use_selected_point_feature,
            post_encoder_input_dims=cfg.post_encoder_input_dims,
            flow_direction=cfg.flow_direction,
        )
    elif cfg.decoder_type in ["flow", "point"]:
        inner_network = AlignedFrameDecoder(
            emb_dims=cfg.emb_dims,
            input_dims=TP_input_dims,
            flow_direction=cfg.flow_direction,
            head_output_type=cfg.decoder_type,
            flow_frame=cfg.flow_frame,
        )

    network = Multimodal_ResidualFlow_DiffEmbTransformer(
        residualflow_diffembtransformer=inner_network,
        gumbel_temp=cfg.gumbel_temp,
        freeze_residual_flow=cfg.freeze_residual_flow,
        center_feature=cfg.center_feature,
        freeze_z_embnn=cfg.freeze_z_embnn,
        division_smooth_factor=cfg.division_smooth_factor,
        add_smooth_factor=cfg.add_smooth_factor,
        conditioning=cfg.conditioning,
        latent_z_linear_size=cfg.latent_z_linear_size,
        taxpose_centering=cfg.taxpose_centering,
        use_action_z=cfg.use_action_z,
        pzY_encoder_type=cfg.pzY_encoder_type,
        pzY_dropout_goal_emb=cfg.pzY_dropout_goal_emb,
        pzY_transformer=cfg.pzY_transformer,
        pzY_transformer_embnn_dims=cfg.pzY_transformer_embnn_dims,
        pzY_transformer_emb_dims=cfg.pzY_transformer_emb_dims,
        pzY_input_dims=cfg.pzY_input_dims,
        pzY_embedding_routine=cfg.pzY_embedding_routine,
        pzY_embedding_option=cfg.pzY_embedding_option,
        hybrid_cond_logvar_limit=cfg.hybrid_cond_logvar_limit,
        latent_z_cond_logvar_limit=cfg.latent_z_cond_logvar_limit,
    )

    model = EquivarianceTrainingModule(  # Is this the trainer ???
        network,
        lr=cfg.lr,
        image_log_period=cfg.image_logging_period,
        flow_supervision=cfg.flow_supervision,
        point_loss_type=cfg.point_loss_type,
        action_weight=cfg.action_weight,
        anchor_weight=cfg.anchor_weight,
        displace_weight=cfg.displace_weight,
        consistency_weight=cfg.consistency_weight,
        smoothness_weight=cfg.smoothness_weight,
        rotation_weight=cfg.rotation_weight,
        # latent_weight=cfg.latent_weight,
        weight_normalize=cfg.weight_normalize,
        softmax_temperature=cfg.softmax_temperature,
        vae_reg_loss_weight=cfg.vae_reg_loss_weight,
        sigmoid_on=cfg.sigmoid_on,
        min_err_across_racks_debug=cfg.min_err_across_racks_debug,
        error_mode_2rack=cfg.error_mode_2rack,
        n_samples=cfg.pzY_n_samples,
        get_errors_across_samples=cfg.pzY_get_errors_across_samples,
        use_debug_sampling_methods=cfg.pzY_use_debug_sampling_methods,
        return_flow_component=cfg.return_flow_component,
        plot_encoder_distribution=cfg.plot_encoder_distribution,
        joint_infonce_loss_weight=cfg.pzY_joint_infonce_loss_weight,
        spatial_distance_regularization_type=cfg.spatial_distance_regularization_type,
        spatial_distance_regularization_weight=cfg.spatial_distance_regularization_weight,
        hybrid_cond_regularize_all=cfg.hybrid_cond_regularize_all,
        pzY_taxpose_infonce_loss_weight=cfg.pzY_taxpose_infonce_loss_weight,
        pzY_taxpose_occ_infonce_loss_weight=cfg.pzY_taxpose_occ_infonce_loss_weight,
        decoder_type=cfg.decoder_type,
        flow_frame=cfg.flow_frame,
        compute_rpdiff_min_errors=cfg.compute_rpdiff_min_errors,
        rpdiff_descriptions_path=cfg.rpdiff_descriptions_path,
    )

    if (
        not cfg.pzX_adversarial
        and not cfg.joint_train_prior
        and cfg.init_cond_x
        and (not cfg.freeze_embnn or not cfg.freeze_residual_flow)
    ):
        raise ValueError("YOU PROBABLY DIDN'T MEAN TO DO JOINT TRAINING")
    if not cfg.joint_train_prior and cfg.init_cond_x and cfg.checkpoint_file is None:
        raise ValueError(
            "YOU PROBABLY DIDN'T MEAN TO TRAIN BOTH P(Z|X) AND P(Z|Y) FROM SCRATCH"
        )

    if cfg.init_cond_x:  # This is true
        network_cond_x = Multimodal_ResidualFlow_DiffEmbTransformer_WithPZCondX(
            residualflow_embnn=network,
            encoder_type=cfg.pzcondx_encoder_type,
            shuffle_for_pzX=cfg.shuffle_for_pzX,
            use_action_z=cfg.use_action_z,
            pzX_transformer=cfg.pzX_transformer,
            pzX_transformer_embnn_dims=cfg.pzX_transformer_embnn_dims,
            pzX_transformer_emb_dims=cfg.pzX_transformer_emb_dims,
            pzX_input_dims=cfg.pzX_input_dims,
            pzX_dropout_goal_emb=cfg.pzX_dropout_goal_emb,
            hybrid_cond_pzX_sample_latent=cfg.hybrid_cond_pzX_sample_latent,
            pzX_embedding_routine=cfg.pzX_embedding_routine,
            pzX_embedding_option=cfg.pzX_embedding_option,
        )

        model_cond_x = EquivarianceTrainingModule_WithPZCondX(
            network_cond_x,
            model,
            goal_emb_cond_x_loss_weight=cfg.goal_emb_cond_x_loss_weight,
            joint_train_prior=cfg.joint_train_prior,
            freeze_residual_flow=cfg.freeze_residual_flow,
            freeze_z_embnn=cfg.freeze_z_embnn,
            freeze_embnn=cfg.freeze_embnn,
            n_samples=cfg.pzX_n_samples,
            get_errors_across_samples=cfg.pzX_get_errors_across_samples,
            use_debug_sampling_methods=cfg.pzX_use_debug_sampling_methods,
            plot_encoder_distribution=cfg.plot_encoder_distribution,
            pzX_use_pzY_z_samples=cfg.pzX_use_pzY_z_samples,
            goal_emb_cond_x_loss_type=cfg.goal_emb_cond_x_loss_type,
            joint_infonce_loss_weight=cfg.pzX_joint_infonce_loss_weight,
            spatial_distance_regularization_type=cfg.spatial_distance_regularization_type,
            spatial_distance_regularization_weight=cfg.spatial_distance_regularization_weight,
            overwrite_loss=cfg.pzX_overwrite_loss,
            pzX_adversarial=cfg.pzX_adversarial,
            hybrid_cond_pzX_regularize_type=cfg.hybrid_cond_pzX_regularize_type,
            hybrid_cond_pzX_sample_latent=cfg.hybrid_cond_pzX_sample_latent,
        )

        if cfg.pzX_adversarial:  # This is False, so we don't use adversarial training
            discriminator = Discriminator(
                encoder_type=cfg.discriminator_encoder_type,
                input_dims=cfg.discriminator_input_dims,
                emb_dims=cfg.discriminator_emb_dims,
                transformer_emb_dims=cfg.discriminator_transformer_emb_dims,
                mlp_hidden_dims=cfg.discriminator_mlp_hidden_dims,
                last_sigmoid=cfg.discriminator_last_sigmoid,
            )

            adversarial_model_cond_x = (
                AdversarialEquivarianceTrainingModule_WithPZCondX(
                    model_cond_x=model_cond_x,
                    discriminator=discriminator,
                    lr=cfg.lr,
                    image_log_period=cfg.image_logging_period,
                    gradient_clipping=gradient_clipping,
                    generator_loss_weight=cfg.generator_loss_weight,
                    discriminator_loss_weight=cfg.discriminator_loss_weight,
                    freeze_taxpose=cfg.pzX_adversarial_freeze_taxpose,
                )
            )

            adversarial_model_cond_x.cuda()
            adversarial_model_cond_x.train()

        else:
            model_cond_x.cuda()
            model_cond_x.train()
    else:
        model.cuda()
        model.train()

    if cfg.checkpoint_file is not None:
        print("loaded checkpoint from")
        print(cfg.checkpoint_file)
        if not cfg.load_cond_x:
            model.load_state_dict(torch.load(cfg.checkpoint_file)["state_dict"])

            if cfg.init_cond_x and cfg.load_pretraining_for_conditioning:
                if cfg.checkpoint_file_action is not None:
                    if model_cond_x.model_with_cond_x.encoder_type == "1_dgcnn":
                        raise NotImplementedError()
                    elif model_cond_x.model_with_cond_x.encoder_type == "2_dgcnn":
                        model_cond_x.model_with_cond_x.p_z_cond_x_embnn_action.conv5 = (
                            nn.Conv2d(512, 512, kernel_size=1, bias=False)
                        )
                        model_cond_x.model_with_cond_x.p_z_cond_x_embnn_action.bn5 = (
                            nn.BatchNorm2d(512)
                        )
                        model_cond_x.model_with_cond_x.p_z_cond_x_embnn_action.load_state_dict(
                            torch.load(cfg.checkpoint_file_action)["embnn_state_dict"]
                        )
                        model_cond_x.model_with_cond_x.p_z_cond_x_embnn_action.conv5 = (
                            nn.Conv2d(512, TP_input_dims - 3, kernel_size=1, bias=False)
                        )
                        model_cond_x.model_with_cond_x.p_z_cond_x_embnn_action.bn5 = (
                            nn.BatchNorm2d(TP_input_dims - 3)
                        )
                        print("----Action Pretraining for p(z|X) Loaded!----")
                    else:
                        raise ValueError()
                if cfg.checkpoint_file_anchor is not None:
                    if model_cond_x.model_with_cond_x.encoder_type == "1_dgcnn":
                        raise NotImplementedError()
                    elif model_cond_x.model_with_cond_x.encoder_type == "2_dgcnn":
                        print("--Not loading p(z|X) for anchor for now--")
                        # model_cond_x.model_with_cond_x.p_z_cond_x_embnn_anchor.conv5 = nn.Conv2d(512, 512, kernel_size=1, bias=False)
                        # model_cond_x.model_with_cond_x.p_z_cond_x_embnn_anchor.bn5 = nn.BatchNorm2d(512)
                        # model_cond_x.model_with_cond_x.p_z_cond_x_embnn_anchor.load_state_dict(
                        #     torch.load(cfg.checkpoint_file_anchor)['embnn_state_dict'])
                        # model_cond_x.model_with_cond_x.p_z_cond_x_embnn_anchor.conv5 = nn.Conv2d(512, TP_input_dims-3, kernel_size=1, bias=False)
                        # model_cond_x.model_with_cond_x.p_z_cond_x_embnn_anchor.bn5 = nn.BatchNorm2d(TP_input_dims-3)
                        # print("--Anchor Pretraining for p(z|X) Loaded!--")
                    else:
                        raise ValueError()
        else:
            model_cond_x.load_state_dict(torch.load(cfg.checkpoint_file)["state_dict"])

    else:
        if cfg.checkpoint_file_action is not None:
            if cfg.load_pretraining_for_taxpose:
                model.model.tax_pose.emb_nn_action.conv1 = nn.Conv2d(
                    3 * 2, 64, kernel_size=1, bias=False
                )
                model.model.tax_pose.emb_nn_action.load_state_dict(
                    torch.load(cfg.checkpoint_file_action)["embnn_state_dict"]
                )
                model.model.tax_pose.emb_nn_action.conv1 = nn.Conv2d(
                    TP_input_dims * 2, 64, kernel_size=1, bias=False
                )
                print(
                    "-----------------------Pretrained EmbNN Action Model Loaded!-----------------------"
                )
            if cfg.load_pretraining_for_conditioning:
                if not cfg.init_cond_x:
                    print("---Not Loading p(z|Y) Pretraining For Now---")
                    # model.model.emb_nn_objs_at_goal.conv5 = nn.Conv2d(512, 512, kernel_size=1, bias=False)
                    # model.model.emb_nn_objs_at_goal.bn5 = nn.BatchNorm2d(512)
                    # model.model.emb_nn_objs_at_goal.load_state_dict(
                    #         torch.load(cfg.checkpoint_file_action)['embnn_state_dict'])
                    # model.model.emb_nn_objs_at_goal.conv5 = nn.Conv2d(512, TP_input_dims-3, kernel_size=1, bias=False)
                    # model.model.emb_nn_objs_at_goal.bn5 = nn.BatchNorm2d(TP_input_dims-3)
                    # print("----Action Pretraining for p(z|Y) Loaded!----")

        if cfg.checkpoint_file_anchor is not None:
            if cfg.load_pretraining_for_taxpose:
                model.model.tax_pose.emb_nn_anchor.conv1 = nn.Conv2d(
                    3 * 2, 64, kernel_size=1, bias=False
                )
                model.model.tax_pose.emb_nn_anchor.load_state_dict(
                    torch.load(cfg.checkpoint_file_anchor)["embnn_state_dict"]
                )
                model.model.tax_pose.emb_nn_anchor.conv1 = nn.Conv2d(
                    TP_input_dims * 2, 64, kernel_size=1, bias=False
                )
                print(
                    "-----------------------Pretrained EmbNN Anchor Model Loaded!-----------------------"
                )
            if cfg.load_pretraining_for_conditioning:
                if not cfg.init_cond_x:
                    print("---Not Loading p(z|Y) Pretraining For Now---")
                    # if cfg.checkpoint_file_action is None:
                    #     model.model.emb_nn_objs_at_goal.conv5 = nn.Conv2d(512, 512, kernel_size=1, bias=False)
                    #     model.model.emb_nn_objs_at_goal.bn5 = nn.BatchNorm2d(512)
                    #     model.model.emb_nn_objs_at_goal.load_state_dict(
                    #             torch.load(cfg.checkpoint_file_action)['embnn_state_dict'])
                    #     model.model.emb_nn_objs_at_goal.conv5 = nn.Conv2d(512, TP_input_dims-3, kernel_size=1, bias=False)
                    #     model.model.emb_nn_objs_at_goal.bn5 = nn.BatchNorm2d(TP_input_dims-3)
                    #     print("----Anchor Pretraining for p(z|Y) Loaded! (because action pretraining is not present)----")

    if cfg.init_cond_x:
        if cfg.pzX_adversarial:
            return trainer, adversarial_model_cond_x, dm
        return trainer, model_cond_x, dm  # !!! This is the return line
    else:
        return trainer, model, dm


@hydra.main(config_path="../../configs/training", config_name="suggester")
def main(cfg):
    trainer, model, dm = setup_main(cfg)

    restarts = 0
    while restarts < 1:
        trainer.fit(model, dm)

        if True or trainer.current_epoch > trainer.max_epochs:
            # This doesn't need a restart. it finished gracefully
            return

        print(
            f"\nTrainer finished. Restarting because current epoch {trainer.current_epoch} is less than max epochs {trainer.max_epochs}"
        )

        # Get the latest checkpoint
        folder = os.path.join(
            trainer.logger.experiment.dir, "..", "..", "..", "residual_flow_occlusion"
        )
        if not os.path.isdir(folder):
            print(f"\nDidn't find the checkpoint folder in {folder}. Quitting.")
            return
        subfolder = os.listdir(folder)[0]
        folder = os.path.join(folder, subfolder, "checkpoints")
        checkpoints = [f for f in os.listdir(folder) if f.startswith("epoch=")]
        if len(checkpoints) == 0:
            print(
                f"\nDidn't find a checkpoint file in {folder}. Assuming the script crashed before the first save. Quitting."
            )
            return

        checkpoint = checkpoints[0]

        del trainer
        del model
        del dm
        import gc

        gc.collect()

        checkpoint_file = os.path.join(folder, checkpoint)
        print("\nRestarting script from latest checkpoint:", checkpoint_file)
        cfg.checkpoint_file = checkpoint_file
        trainer, model, dm = setup_main(cfg)

    print(f"\nAlready restarted {restarts} times. Quitting")


if __name__ == "__main__":
    torch.set_float32_matmul_precision("high")
    torch.autograd.set_detect_anomaly(True)
    torch.cuda.empty_cache()
    torch.multiprocessing.set_sharing_strategy("file_system")
    main()
