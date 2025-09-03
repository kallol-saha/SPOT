import os
from typing import Optional, Tuple, Union

import numpy as np
import pytorch_lightning as pl
import torch
import torch.nn as nn
import torch.optim as optim
import spot.submodules.pyg_libs.src.rpad.pyg.nets.pointnet2 as pnp
from torch_geometric.data import Batch, Data
from spot.models.mlp import MLP


class ObjectSuggesterClassifier(nn.Module):
    """
    Object suggester classifier for point cloud planning.

    Key components:
    1. Feature extraction using PointNet++
    2. Transformation network to output a prediction
    """

    def __init__(self, latent_dim: int = 1024):
        super().__init__()

        self.feature_extractor = pnp.PN2Encoder(
            in_dim=1,  # Mask on each point serves as an object query
            out_dim=latent_dim,
        )

        self.mlp = MLP(
            input_size=latent_dim,
            hidden_sizes=[256, 128, 64],
            output_size=1,
        )
        self.sigmoid = nn.Sigmoid()

    def forward(self, data: Data) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        data:
            data.x contains the query mask (1s for the object we're querying, 0s everywhere else)
            data.pos contains the point cloud x, y, z
        """
        # "data" has an extra label attribute for the gt label
        # data_no_label = Data(x=data.x, pos=data.pos, batch=data.batch)
        features = self.feature_extractor(data)
        output = self.mlp(features)

        return self.sigmoid(output)

    def predict(
        self,
        pcd: Union[torch.Tensor, np.ndarray] = None,
        pcd_seg: Union[torch.Tensor, np.ndarray] = None,
        seg_id: int = None,
        data: Optional[Data] = None,
        device: str = "cuda",
    ) -> torch.Tensor:
        """Predict action and anchor objects for the given point cloud state.

        Args:
            pcd: (N, 3) point cloud
            query: (N,) features

        Returns:
            torch.Tensor: probability of moving the query object
        """
        self.eval()

        if data is None:
            if type(pcd) != torch.Tensor:
                pcd = torch.from_numpy(pcd).to(device)
                pcd_seg = torch.from_numpy(pcd_seg).to(device)

            # Create query mask
            query_mask = torch.zeros_like(pcd_seg, device=device, dtype=torch.float32)
            query_mask[pcd_seg == seg_id] = 1.0
            query_mask = query_mask.unsqueeze(-1)  # (N, 1)

            data = Data(x=query_mask, pos=pcd)

        with torch.no_grad():
            batch = Batch.from_data_list([data])
            batch = batch.to(device)

            features = self.feature_extractor(batch)
            out = self.sigmoid(self.mlp(features))
        return out


class ObjectSuggesterClassifierLightningModule(pl.LightningModule):
    """
    PyTorch Lightning module for training.
    Handles training and validation logic.
    """

    def __init__(
        self,
        learning_rate: float = 1e-3,
    ):
        super().__init__()
        self.save_hyperparameters()

        self.model = ObjectSuggesterClassifier()
        self.loss_fn = nn.BCELoss()

        self.best_train_loss = float("inf")
        self.best_val_loss = float("inf")

    def forward(self, data: Data):
        return self.model(data)

    def predict(
        self,
        pcd: Union[torch.Tensor, np.ndarray] = None,
        pcd_seg: Union[torch.Tensor, np.ndarray] = None,
        seg_id: int = None,
        data: Optional[Data] = None,
    ) -> torch.Tensor:
        return self.model.predict(
            pcd=pcd, pcd_seg=pcd_seg, seg_id=seg_id, data=data, device=self.device
        )

    def training_step(self, batch: Batch, batch_idx: int) -> torch.Tensor:
        """
        Compute and log training loss
        """
        self.train()

        predicted = self(batch)
        B = predicted.shape[0]
        labels = batch.label.unsqueeze(1)

        loss = self.loss_fn(predicted, labels)
        self.log(
            "train_loss",
            loss,
            prog_bar=True,
            logger=True,
            batch_size=B,
        )

        if loss < self.best_train_loss:
            self.best_train_loss = loss
        self.log(
            "best_train_loss",
            self.best_train_loss,
            logger=True,
            batch_size=B,
        )

        return loss

    def validation_step(self, batch: Batch, batch_idx: int) -> None:
        """
        Compute and log validation metrics
        """
        self.eval()

        predicted = self(batch)
        B = predicted.shape[0]
        labels = batch.label.unsqueeze(1)

        val_loss = self.loss_fn(predicted, labels)
        self.log(
            "val_loss",
            val_loss,
            prog_bar=True,
            logger=True,
            batch_size=B,
        )

        if val_loss < self.best_val_loss:
            self.best_val_loss = val_loss
        self.log(
            "best_val_loss",
            self.best_val_loss,
            logger=True,
            batch_size=B,
        )

        return val_loss

    def configure_optimizers(self):
        """
        Configure optimizer for training
        """
        optimizer = optim.Adam(self.parameters(), lr=self.hparams.learning_rate)
        return optimizer

def load_model(
    folder_name: str, parent_folder: str = "assets/weights"
) -> ObjectSuggesterClassifier:
    folder = os.path.join(parent_folder, folder_name)

    # Take the last checkpoint in the folder
    ckpts = sorted(c for c in os.listdir(folder) if c.endswith(".ckpt"))
    ckpt_file_path = os.path.join(folder, ckpts[-1])

    model = ObjectSuggesterClassifier()
    model.cuda()
    model.load_state_dict(torch.load(ckpt_file_path)["state_dict"])
    model.eval()

    return model
