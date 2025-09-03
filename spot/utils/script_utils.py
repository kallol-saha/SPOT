import os
import pathlib
from typing import Dict, List, Sequence, Union, cast

import torch
import torch.utils._pytree as pytree
import torchvision as tv
import wandb
from lightning.pytorch import Callback
from pytorch_lightning.loggers import WandbLogger

PROJECT_ROOT = str(pathlib.Path(__file__).parent.parent.parent.parent.resolve())


def create_model(image_size, num_classes, model_cfg):
    if model_cfg.name == "vit":
        return tv.models.VisionTransformer(
            image_size=image_size,
            num_classes=num_classes,
            hidden_dim=model_cfg.hidden_dim,
            num_heads=model_cfg.num_heads,
            num_layers=model_cfg.num_layers,
            patch_size=model_cfg.patch_size,
            representation_size=model_cfg.representation_size,
            mlp_dim=model_cfg.mlp_dim,
            dropout=model_cfg.dropout,
        )
    else:
        raise ValueError("not a valid model name")


# This matching function
def match_fn(dirs: Sequence[str], extensions: Sequence[str], root: str = PROJECT_ROOT):
    def _match_fn(path: pathlib.Path):
        in_dir = any([str(path).startswith(os.path.join(root, d)) for d in dirs])

        if not in_dir:
            return False

        if not any([str(path).endswith(e) for e in extensions]):
            return False

        return True

    return _match_fn


TorchTree = Dict[str, Union[torch.Tensor, "TorchTree"]]


def flatten_outputs(outputs: List[TorchTree]) -> TorchTree:
    """Flatten a list of dictionaries into a single dictionary."""

    # Concatenate all leaf nodes in the trees.
    flattened_outputs = [pytree.tree_flatten(output) for output in outputs]
    flattened_list = [o[0] for o in flattened_outputs]
    flattened_spec = flattened_outputs[0][1]  # Spec definitely should be the same...
    cat_flat = [torch.cat(x) for x in list(zip(*flattened_list))]
    output_dict = pytree.tree_unflatten(cat_flat, flattened_spec)
    return cast(TorchTree, output_dict)


class LogPredictionSamplesCallback(Callback):
    def __init__(self, logger: WandbLogger):
        self.logger = logger

    def on_validation_batch_end(
        self, trainer, pl_module, outputs, batch, batch_idx, dataloader_idx=0
    ):
        """Called when the validation batch ends."""

        # `outputs` comes from `LightningModule.validation_step`
        # which corresponds to our model predictions in this case

        # Let's log 20 sample image predictions from the first batch
        if batch_idx == 0:
            n = 20
            x, y = batch
            images = [img for img in x[:n]]
            outs = outputs["preds"][:n].argmax(dim=1)
            captions = [
                f"Ground Truth: {y_i} - Prediction: {y_pred}"
                for y_i, y_pred in zip(y[:n], outs)
            ]

            # Option 1: log images with `WandbLogger.log_image`
            self.logger.log_image(key="sample_images", images=images, caption=captions)

            # Option 2: log images and predictions as a W&B Table
            columns = ["image", "ground truth", "prediction"]
            data = [
                [wandb.Image(x_i), y_i, y_pred]
                for x_i, y_i, y_pred in list(zip(x[:n], y[:n], outs))
            ]
            self.logger.log_table(key="sample_table", columns=columns, data=data)
