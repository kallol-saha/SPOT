import os
import pathlib
import warnings

import hydra
import pytorch_lightning as pl
import torch
import torch_geometric.loader as tgl
from pytorch_lightning.callbacks import Callback
from pytorch_lightning.loggers import WandbLogger

from spot.datasets.obj_suggester_dataset import ObjectSuggesterClassifierDataset
from spot.models.object_suggester_classifier import (
    ObjectSuggesterClassifierLightningModule,
)
from spot.utils.path_utils import resolve_config_paths

# Suppress a UserWarning from torch geometric
warnings.filterwarnings("ignore", message="TypedStorage is deprecated")


class SaverCallbackModel(Callback):
    """
    Save a checkpoint every N steps, instead of Lightning's default that checkpoints
    based on validation loss.
    """

    def __init__(self, save_freq=1000):
        self.save_freq = save_freq
        self.prev_path = None

    def on_train_batch_end(
        self, trainer, pl_module, outputs, batch, batch_idx
    ):  # , dataloader_idx):
        """Check if we should save a checkpoint after every train batch"""
        epoch = trainer.current_epoch
        global_step = trainer.global_step

        if global_step % self.save_freq == 0 and global_step > 100:
            filename = f"epoch_{epoch}_global_step_{global_step}.ckpt"
            ckpt_path_embnn = os.path.join(
                trainer.checkpoint_callback.dirpath, filename
            )
            if not os.path.isdir(trainer.checkpoint_callback.dirpath):
                os.makedirs(trainer.checkpoint_callback.dirpath)

            # Importantly, saves the state dict of the model INSIDE the pl module
            torch.save({"state_dict": pl_module.model.state_dict()}, ckpt_path_embnn)
            if self.prev_path is not None:
                self.prev_path.unlink()
                self.prev_path = pathlib.Path(ckpt_path_embnn)


@hydra.main(config_path="../../configs/training", config_name="object_suggester")
def train(cfg):
    """Main training script for object suggester"""
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
    full_dataset = ObjectSuggesterClassifierDataset(
        cfg.dataset_root,
        cfg.object_ids,
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
    module = ObjectSuggesterClassifierLightningModule(
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
        model=module, train_dataloaders=train_loader, val_dataloaders=val_loader
    )

    # Clean up training
    if cfg.wandb:
        logger.experiment.finish()


if __name__ == "__main__":
    torch.set_float32_matmul_precision("high")
    torch.autograd.set_detect_anomaly(True)
    torch.cuda.empty_cache()
    torch.multiprocessing.set_sharing_strategy("file_system")

    train()
