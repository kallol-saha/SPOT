from placement_suggester.models.transformer_flow import ResidualFlow_DiffEmbTransformer
from placement_suggester.models.multimodal_transformer_flow import Multimodal_ResidualFlow_DiffEmbTransformer, Multimodal_ResidualFlow_DiffEmbTransformer_WithPZCondX
from placement_suggester.training.flow_equivariance_training_module_nocentering_eval_init import EquivarianceTestingModule
from placement_suggester.training.flow_equivariance_training_module_nocentering_multimodal import EquivarianceTrainingModule, EquivarianceTrainingModule_WithPZCondX
import torch

def load_model(place_checkpoint_file, has_pzX, conditioning="pos_delta_l2norm", return_flow_component=None, cfg=None):
    if cfg is not None:
        print("Loading model with architecture specified in configs")
        TP_input_dims = Multimodal_ResidualFlow_DiffEmbTransformer.TP_INPUT_DIMS[cfg.conditioning]
        if cfg.conditioning in ["latent_z_linear", "hybrid_pos_delta_l2norm", "hybrid_pos_delta_l2norm_global"]:
            TP_input_dims += cfg.latent_z_linear_size # Hacky way to add the dynamic latent z to the input dims


        inner_network = ResidualFlow_DiffEmbTransformer(
            emb_dims=cfg.emb_dims,
            input_dims=TP_input_dims,
            emb_nn=cfg.emb_nn,
            return_flow_component=cfg.return_flow_component,
            center_feature=cfg.center_feature,
            inital_sampling_ratio=cfg.inital_sampling_ratio,
            pred_weight=cfg.pred_weight,
            freeze_embnn=cfg.freeze_embnn,
            conditioning_size=cfg.latent_z_linear_size if cfg.conditioning in ["latent_z_linear_internalcond", "hybrid_pos_delta_l2norm_internalcond", "hybrid_pos_delta_l2norm_global_internalcond"] else 0,
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
        )

        place_nocond_model = EquivarianceTrainingModule(
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
            #latent_weight=cfg.latent_weight,
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
        
        if has_pzX:
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

            place_model = EquivarianceTrainingModule_WithPZCondX(
                network_cond_x,
                place_nocond_model,
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
        else:
            place_model = place_nocond_model
    else:
        print("WARNING: No configs for the model are specified. Using default configs to load the model")
        inner_network = ResidualFlow_DiffEmbTransformer(
                    emb_nn='dgcnn', return_flow_component=return_flow_component, center_feature=True,
                    inital_sampling_ratio=1, input_dims=4)
        place_nocond_network = Multimodal_ResidualFlow_DiffEmbTransformer(
                            inner_network, gumbel_temp=1, center_feature=True, conditioning=conditioning)
        place_nocond_model = EquivarianceTrainingModule(
            place_nocond_network,
            lr=1e-4,
            image_log_period=100,
            weight_normalize='softmax', #'l1',
            softmax_temperature=1
        )
        
        if has_pzX:
            place_network = Multimodal_ResidualFlow_DiffEmbTransformer_WithPZCondX(
                            place_nocond_network, encoder_type="2_dgcnn", sample_z=False)
            place_model = EquivarianceTrainingModule_WithPZCondX(
                place_network,
                place_nocond_model,
            )
        else:
            place_model = place_nocond_model

    place_model.cuda()

    if place_checkpoint_file is not None:
        place_model.load_state_dict(torch.load(place_checkpoint_file)['state_dict'])
    else:
        print("WARNING: NO CHECKPOINT FILE SPECIFIED. THIS IS A DEBUG RUN WITH RANDOM WEIGHTS")
    return place_model

def load_merged_model(place_checkpoint_file_pzY, place_checkpoint_file_pzX, conditioning="pos_delta_l2norm", return_flow_component=None, cfg=None):
    if place_checkpoint_file_pzY is not None:
        pzY_model = load_model(place_checkpoint_file_pzY, has_pzX=False, conditioning=conditioning, return_flow_component=return_flow_component, cfg=cfg)
    
        if place_checkpoint_file_pzX is not None:
            pzX_model = load_model(place_checkpoint_file_pzX, has_pzX=True, conditioning=conditioning, return_flow_component=return_flow_component, cfg=cfg)
            pzX_model.model.tax_pose = pzY_model.model.tax_pose
            return pzX_model
        else:
            return pzY_model
    else:
        if place_checkpoint_file_pzX is not None:
            pzX_model = load_model(place_checkpoint_file_pzX, has_pzX=True, conditioning=conditioning, return_flow_component=return_flow_component, cfg=cfg)
            return pzX_model
        else:
            raise ValueError("No checkpoint file specified for either pzY or pzX. Cannot load a model")
