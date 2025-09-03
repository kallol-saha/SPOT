import os
import warnings

import hydra
import pytorch_lightning as pl
import torch
import torch_geometric.loader as tgl
from placement_suggester.utils.callbacks import SaverCallbackModel
from pytorch_lightning.loggers import WandbLogger
from hydra import compose, initialize
from hydra.core.global_hydra import GlobalHydra

from spot.datasets.mde_datasets import (
    MDEChamferDistanceDataset,
    MDEClassifierDataset,
    MDEDataset,
    MDETableBussingClassifierDataset,
    MDETableBussingDataset,
)
from spot.models.mde_model import PointCloudMDELightningModule
from spot.utils.path_utils import resolve_config_paths

# Suppress a UserWarning from torch geometric
warnings.filterwarnings("ignore", message="TypedStorage is deprecated")

@hydra.main(config_path="../../configs/training", config_name="model_deviation_estimator")
def train_mde(cfg):
    """Main training script for MDE"""
    # Resolve relative paths to absolute paths
    cfg = resolve_config_paths(cfg)
    
    pl.seed_everything(cfg.seed, workers=True)

    if cfg.wandb:
        if cfg.resume_id is None:
            logger = WandbLogger(project=cfg.experiment, group=cfg.wandb_group)
        else:
            logger = WandbLogger(
                project=cfg.experiment,
                group=cfg.wandb_group,
                id=cfg.resume_id,
                resume="must",
            )

        logger.log_hyperparams(cfg)
        logger.log_hyperparams({"working_dir": os.getcwd()})
    else:
        logger = None

    # Load dataset
    if cfg.env == "blocks":
        if cfg.classifier:
            full_dataset = MDEClassifierDataset(
                cfg.dataset_root,
                mean_centered=cfg.mean_centered,
                object_ids=cfg.object_ids,
            )
        elif cfg.chamfer_deviation:
            full_dataset = MDEChamferDistanceDataset(
                cfg.dataset_root,
                object_ids=cfg.object_ids,
                eps=cfg.epsilon,
                mean_centered=cfg.mean_centered,
                clip_min=cfg.clip_min,
                clip_max=cfg.clip_max,
                scaler=cfg.scaler,
            )
        else:
            full_dataset = MDEDataset(
                cfg.dataset_root,
                mean_centered=cfg.mean_centered,
                object_ids=cfg.object_ids,
                clip_min=cfg.clip_min,
                clip_max=cfg.clip_max,
                scaler=cfg.scaler,
            )
    elif cfg.env == "table_bussing":
        if cfg.classifier:
            full_dataset = MDETableBussingClassifierDataset(
                cfg.dataset_root,
                mean_centered=cfg.mean_centered,
                object_ids=cfg.object_ids,
            )
        else:
            full_dataset = MDETableBussingDataset(
                cfg.dataset_root,
                cfg.object_ids,
                cfg.epsilon,
                clip_min=cfg.clip_min,
                clip_max=cfg.clip_max,
                scaler=cfg.scaler,
            )

    # Calculate lengths for 80-20 split
    total_size = len(full_dataset)
    train_size = int(0.8 * total_size)
    val_size = total_size - train_size

    train_dset, val_dset = torch.utils.data.random_split(
        full_dataset,
        [train_size, val_size],
        generator=torch.Generator().manual_seed(cfg.seed),
    )

    # Create data loaders
    train_loader = tgl.DataLoader(
        train_dset, batch_size=cfg.batch_size, shuffle=True, num_workers=cfg.num_workers
    )
    val_loader = tgl.DataLoader(
        val_dset, batch_size=cfg.batch_size, shuffle=False, num_workers=cfg.num_workers
    )

    # Initialize Lightning module
    mde_module = PointCloudMDELightningModule(
        latent_dim=cfg.latent_dim,
        classifier=cfg.classifier,
        loss_fn=cfg.loss_fn,
        c1=cfg.mse_c1,
        c2=cfg.mse_c2,
        learning_rate=cfg.learning_rate,
    )

    # Setup trainer
    trainer = pl.Trainer(
        logger=logger,
        default_root_dir=cfg.checkpoint_dir,
        callbacks=[SaverCallbackModel(save_freq=cfg.ckpt_save_freq)],
        max_epochs=cfg.max_epochs,
        check_val_every_n_epoch=cfg.check_val_every_n_epoch,
        accelerator=cfg.accelerator,
        devices=cfg.devices,
        deterministic="warn",
        log_every_n_steps=5,
    )

    # Train the model
    trainer.fit(
        model=mde_module, train_dataloaders=train_loader, val_dataloaders=val_loader
    )

    # Clean up training
    if cfg.wandb:
        logger.experiment.finish()


if __name__ == "__main__":
    torch.set_float32_matmul_precision("high")
    torch.autograd.set_detect_anomaly(True)
    torch.cuda.empty_cache()
    torch.multiprocessing.set_sharing_strategy("file_system")

    train_mde()
