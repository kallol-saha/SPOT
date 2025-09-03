import os
from typing import Optional

import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend to prevent figure display
import matplotlib.pyplot as plt
import numpy as np
import pytorch_lightning as pl
import torch
import torch.nn as nn
import torch.optim as optim

# TODO: Fork all submodules, make our changes, update submodule references
import spot.submodules.pyg_libs.src.rpad.pyg.nets.pointnet2 as pnp
import wandb
from torch_geometric.data import Batch, Data


def plot_gt_vs_predicted(predicted: torch.Tensor, target: torch.Tensor):
    """
    Create a scatter plot comparing predicted vs target values.
    Uses non-interactive matplotlib backend to prevent blocking execution.
    
    Args:
        predicted: Predicted values tensor
        target: Target/ground truth values tensor
        
    Returns:
        matplotlib figure object (non-interactive)
    """
    target = target.cpu().detach().numpy()
    predicted = predicted.cpu().detach().numpy()

    fig, ax = plt.subplots()
    ax.plot(target, predicted, "b.")
    ax.axline((0, 0), slope=1)
    ax.set_xlabel("Target")
    ax.set_ylabel("Predicted")

    return fig


class AsymmetricMSELoss(nn.Module):
    def __init__(self, c1: float, c2: float):
        """
        Asymmetric MSE loss with different penalties for over/under-prediction

        Args:
            c1 (float): Coefficient for underprediction penalty
            c2 (float): Coefficient for overprediction penalty
        """
        super().__init__()
        self.c1 = c1
        self.c2 = c2

    def forward(self, predicted: torch.Tensor, target: torch.Tensor) -> torch.Tensor:
        # Calculate positive parts of differences
        under_pred = torch.maximum(target - predicted, torch.tensor(0.0))
        over_pred = torch.maximum(predicted - target, torch.tensor(0.0))

        # Calculate weighted squared terms
        loss = self.c1 * (under_pred**2) + self.c2 * (over_pred**2)

        # Return mean loss across all elements
        return torch.mean(loss)


class PointCloudMDE(nn.Module):
    """
    Model Deviation Estimator (MDE) for point cloud planning.

    Key components:
    1. Feature extraction using PointNet++
    2. Transformation network to output a prediction
    """

    def __init__(
        self,
        latent_dim: int = 1024,
        classifier: bool = False,
        downsample: bool = False,
    ):
        super().__init__()
        self.classifier = classifier

        # Feature extraction: outputs a single latent vector
        self.feature_extractor = pnp.PN2Encoder(
            in_dim=3,  # delta x, delta y, delta z
            out_dim=latent_dim,
            # Using default encoder params
        )

        modules = [
            nn.Linear(latent_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, 1),
        ]

        if classifier:
            modules.append(nn.Sigmoid())

        # Transformation network
        self.mlp = nn.Sequential(*modules)

    def forward(self, data: Data) -> torch.Tensor:
        features = self.feature_extractor(data)
        output = self.mlp(features)

        return output

    def predict(
        self,
        pcd: np.ndarray,
        action: np.ndarray,
        device: str,
    ) -> torch.Tensor:
        """Predict what happens when applying the action to the point cloud.

        - If regression model, outputs a deviation.
        - If classifier, outputs 0 meaning valid action and 1 meaning invalid action.

        Args:
            pcd: Nx3 point cloud
            action: Nx3 delta x, delta y, delta z indicating the ground truth transform on each point

        Returns:
            torch.Tensor: 1x1 prediction
        """
        assert pcd.shape[0] == action.shape[0]

        pos = torch.tensor(pcd, dtype=torch.float32)
        x = torch.tensor(action, dtype=torch.float32)

        data = Data(x=x, pos=pos)
        batch = Batch.from_data_list([data])
        batch = batch.to(device)

        self.eval()
        with torch.no_grad():
            prediction = self.forward(batch).item()
        return prediction


class PointCloudMDELightningModule(pl.LightningModule):
    """
    PyTorch Lightning module for training the MDE.
    Handles training and validation logic.
    """

    def __init__(
        self,
        latent_dim: int = 1024,
        classifier: bool = False,
        loss_fn: str = "asym_mse",
        c1: Optional[float] = None,
        c2: Optional[float] = None,
        learning_rate: float = 1e-3,
    ):
        super().__init__()
        self.classifier = classifier

        self.save_hyperparameters()

        # Instantiate the model
        self.mde = PointCloudMDE(latent_dim=latent_dim, classifier=classifier)

        # Loss function
        if classifier:
            self.loss_fn = nn.BCELoss()
        else:  # Regression model
            if loss_fn == "asym_mse":
                self.loss_fn = AsymmetricMSELoss(c1=c1, c2=c2)
            elif loss_fn == "mse":
                self.loss_fn = nn.MSELoss()
            else:
                raise ValueError(f"Loss function {loss_fn} not supported")

        self.best_train_loss = float("inf")
        self.best_val_loss = float("inf")

    def forward(self, data: Data) -> torch.Tensor:
        return self.mde(data)

    def predict(
        self,
        pcd: np.ndarray,
        action: np.ndarray,
    ) -> torch.Tensor:
        return self.mde.predict(
            pcd,
            action,
            device=self.device,
        )

    def training_step(self, batch: Batch, batch_idx: int) -> torch.Tensor:
        """
        Compute and log training loss
        """
        self.train()

        predicted = self(batch)
        labels = batch.label.unsqueeze(1)

        loss = self.loss_fn(predicted, labels)
        self.log(
            "train_loss",
            loss,
            prog_bar=True,
            logger=True,
            batch_size=predicted.shape[0],
        )

        if loss < self.best_train_loss:
            self.best_train_loss = loss
        self.log(
            "best_train_loss",
            self.best_train_loss,
            logger=True,
            batch_size=predicted.shape[0],
        )

        if not self.classifier:
            fig = plot_gt_vs_predicted(predicted, labels)
            self.logger.log_image("gt_vs_predicted_train", images=[wandb.Image(fig)])
            plt.close(fig)

        return loss

    def validation_step(self, batch: Batch, batch_idx: int) -> None:
        """
        Compute and log validation metrics
        """
        self.eval()

        predictions = self(batch)
        labels = batch.label.unsqueeze(1)

        val_loss = self.loss_fn(predictions, labels)
        self.log(
            "val_loss",
            val_loss,
            prog_bar=True,
            logger=True,
            batch_size=predictions.shape[0],
        )

        if val_loss < self.best_val_loss:
            self.best_val_loss = val_loss
        self.log(
            "best_val_loss",
            self.best_val_loss,
            logger=True,
            batch_size=predictions.shape[0],
        )

        if not self.classifier:
            fig = plot_gt_vs_predicted(predictions, labels)
            self.logger.log_image("gt_vs_predicted_val", images=[wandb.Image(fig)])
            plt.close(fig)

        return val_loss

    def configure_optimizers(self):
        """
        Configure optimizer for training
        """
        optimizer = optim.Adam(self.parameters(), lr=self.hparams.learning_rate)
        return optimizer


def load_mde_model(
    folder_name: str, parent_folder: str = "assets/weights"
) -> PointCloudMDELightningModule:
    folder = os.path.join(parent_folder, folder_name)

    # Take the last checkpoint in the folder
    ckpts = sorted(c for c in os.listdir(folder) if c.endswith(".ckpt"))
    ckpt_file_path = os.path.join(folder, ckpts[-1])

    model = PointCloudMDELightningModule.load_from_checkpoint(ckpt_file_path)
    model.eval()

    return model
