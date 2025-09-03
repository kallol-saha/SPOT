from dataclasses import dataclass
from typing import Literal

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch_geometric.data import Data
from torch_geometric.nn import PointConv, fps, global_max_pool, knn_interpolate, radius

from rpad.pyg.nets.mlp import MLP, MLPParams


@dataclass
class SAParams:
    # Ratio of points to sample.
    ratio: float

    # Radius for the ball query.
    r: float

    # Parameters for the PointNet MLP.
    net_params: MLPParams

    # Maximum number of neighbors for the ball query to return.
    max_num_neighbors: int = 64


class SAModule(nn.Module):
    def __init__(self, in_chan: int, out_chan: int, p: SAParams):
        """Create a Set Aggregation module from PointNet++.

        Algorithm:
          1) Perform "farthest point sampling", selecting a number of points proportional
             to the specified ratio. i.e. if the input has 10000 points and the ratio is .2,
             we select 2000 points with FPS.
          2) For each selected points, get all points within radius r, up to max_num_neighbors.
          3) Run PointNet (vanilla) on each region independently.

        Args:
            p (SAParams) See above.

        """
        super(SAModule, self).__init__()
        self.ratio = p.ratio
        self.max_num_neighbors = p.max_num_neighbors
        self.r = p.r

        # NOTE: "add_self_loops" for some reason breaks the gradient flow in a batch!!!
        # See my bug at https://github.com/rusty1s/pytorch_geometric/issues/2558
        self.conv = PointConv(
            MLP(in_chan, out_chan, p.net_params), add_self_loops=False
        )

    def forward(self, x, pos, batch):
        if self.ratio == 1.0:
            selected_pos = pos
            selected_batch = batch
        else:
            # Select the points using "Farthest Point Sampling".
            idx = fps(pos, batch, ratio=self.ratio)

            selected_pos = pos[idx]
            selected_batch = batch[idx]

        # Perform a ball query around each of the points.
        row, col = radius(
            pos,
            selected_pos,
            self.r,
            batch,
            selected_batch,
            max_num_neighbors=self.max_num_neighbors,
        )

        # Run PointNet on each point set independently.
        edge_index = torch.stack([col, row], dim=0)
        x = self.conv(x, (pos, selected_pos), edge_index)
        pos, batch = selected_pos, selected_batch

        return x, pos, batch


@dataclass
class GlobalSAParams:
    net_params: MLPParams


class GlobalSAModule(nn.Module):
    def __init__(self, in_chan: int, out_chan: int, p: GlobalSAParams):
        """Module to perform the Global Set Aggregation operation.

        Two steps:
        1) For each point, apply the mlp specified in net_params.
        2) Use the global_max_pool operation to get a final feature vector.

        Args:
            p (GlobalSAParams): Parameters.
        """
        super(GlobalSAModule, self).__init__()
        self.net = MLP(in_chan, out_chan, p.net_params)

    def forward(self, x, pos, batch):
        # Batch application of the MLP.
        x = self.net(torch.cat([x, pos], dim=1))

        # MaxPool operation.
        x = global_max_pool(x, batch)
        pos = pos.new_zeros((x.size(0), 3))
        batch = torch.arange(x.size(0), device=batch.device)
        return x, pos, batch


@dataclass
class FPParams:
    # MLP Parameters for the feature update step.
    net_params: MLPParams

    # Number of neighbors used for knn_interpolate.
    k: int = 3


class FPModule(torch.nn.Module):
    def __init__(self, in_channels: int, out_channels: int, params: FPParams):
        """Feature Propagation Module.

        1) Interpolate each point based on the k nearest embedded points.
        2) Concatenate skip features.
        3) Update feature vectors with another MLP.

        Args:
            params (FPParams): Parameters.

        """
        super().__init__()
        self.k = params.k
        self.net = MLP(in_channels, out_channels, params.net_params)

    def forward(self, x, pos, batch, x_skip, pos_skip, batch_skip):
        # If we need to interpolate, interpolate.
        # Otherwise (i.e. when the sampling ratio is 1.0, we don't need to.
        if pos.shape[0] != pos_skip.shape[0]:
            # Perform the interpolation.
            x = knn_interpolate(x, pos, pos_skip, batch, batch_skip, k=self.k)

        # If we have skip connections concatenate them.
        if x_skip is not None:
            x = torch.cat([x, x_skip], dim=1)

        # Run them
        x = self.net(x)

        return x, pos_skip, batch_skip


@dataclass
class PN2EncoderParams:
    # Layer 1. See SAParams for description.
    sa1: SAParams = SAParams(0.2, 0.2, MLPParams((64, 64)), 64)
    sa1_outdim: int = 128

    # Layer 2. See SAParams for description.
    sa2: SAParams = SAParams(0.25, 0.4, MLPParams((128, 128)), 64)
    sa2_outdim: int = 256

    # Global aggregation. See GlobalSAParams for description.
    gsa: GlobalSAParams = GlobalSAParams(MLPParams((256, 512)))


class PN2Encoder(nn.Module):
    def __init__(
        self,
        in_dim: int = 0,
        out_dim: int = 1024,
        p: PN2EncoderParams = PN2EncoderParams(),
    ):
        """A PointNet++ encoder. Takes in a pointcloud, outputs a single latent vector.

        Args:
            in_dim: The dimensionality of the feature vector attached to each point. i.e. if it's
                    a mask, then in_dim=1. No features => in_dim=1
            out_dim: Dimensionality of the output.
            p: Internal parameters for the network.
        """
        super().__init__()

        # The Set Aggregation modules.
        self.sa1_module = SAModule(in_chan=3 + in_dim, out_chan=p.sa1_outdim, p=p.sa1)
        self.sa2_module = SAModule(
            in_chan=3 + p.sa1_outdim, out_chan=p.sa2_outdim, p=p.sa2
        )
        self.sa3_module = GlobalSAModule(
            in_chan=3 + p.sa2_outdim, out_chan=out_dim, p=p.gsa
        )

    def forward(self, data):
        sa0_out = (data.x, data.pos, data.batch)
        sa1_out = self.sa1_module(*sa0_out)
        sa2_out = self.sa2_module(*sa1_out)
        sa3_out = self.sa3_module(*sa2_out)
        x, pos, batch = sa3_out

        return x


@dataclass
class PN2DenseParams:
    ###########################################################################
    # SET AGGREGATION
    ###########################################################################

    # Layer 1. See SAParams for description.
    sa1: SAParams = SAParams(0.2, 0.2, MLPParams((64, 64), out_act="none"), 64)
    sa1_outdim: int = 128

    # Layer 2. See SAParams for description.
    sa2: SAParams = SAParams(0.25, 0.4, MLPParams((128, 128), out_act="none"), 64)
    sa2_outdim: int = 256

    # Global aggregation. See GlobalSAParams for description.
    gsa: GlobalSAParams = GlobalSAParams(MLPParams((256, 512), out_act="none"))
    gsa_outdim: int = 1024

    ###########################################################################
    # FEATURE PROPAGATION
    # Since this is the decoder, execution happens 3->2->1.
    ###########################################################################

    # Layer 3. See FPParams for description.
    fp3: FPParams = FPParams(MLPParams((256,), out_act="none"), k=1)

    # Layer 2. See FPParams for description.
    fp2: FPParams = FPParams(MLPParams((256,), out_act="none"), k=3)

    # Layer 1. See FPParams for description.
    fp1: FPParams = FPParams(MLPParams((128, 128), out_act="none"), k=3)
    fp1_outdim: int = 128

    # Dimensions of the final 2 linear layers.
    lin1_dim: int = 128
    lin2_dim: int = 128

    # Output layer activation.
    out_act: Literal["none", "softmax", "relu"] = "none"


class PN2Dense(nn.Module):
    def __init__(
        self,
        in_channels: int = 0,
        out_channels: int = 3,
        p: PN2DenseParams = PN2DenseParams(),
    ):
        """The PointNet++ "dense" network architecture, as proposed in the original paper.
        In general, the parameters "p" are architecture or hyperparameter choices for the network.
        The other arguments are structural ones, determining the input and output dimensionality.

        It's a bit of a U-Net architecture, so I've written some automatic wiring to make sure that
        the layers all agree.

        Args:
            in_channels: The number of non-XYZ channels attached to each point. For instance, no additional
                         features would have in_channels=0, RGB features would be in_channels=3, a binary mask
                         would be in_channels=1, etc. For a point cloud passed to the network with N points,
                         the `x` property must be set on the object to a float tensor of shape [N x in_channels].
            out_channels: The dimension of the per-point output channels.
            p: Architecture and hyperparameters for the network. Default is the original set from the paper.
        """
        super().__init__()

        self.in_ch = in_channels
        self.out_ch = out_channels

        # Construct the set aggregation modules. This is the encoder.
        self.sa1 = SAModule(3 + self.in_ch, p.sa1_outdim, p=p.sa1)
        self.sa2 = SAModule(3 + p.sa1_outdim, p.sa2_outdim, p=p.sa2)
        self.sa3 = GlobalSAModule(3 + p.sa2_outdim, p.gsa_outdim, p=p.gsa)

        # The Feature Propagation modules. This is the decoder.
        self.fp3 = FPModule(p.gsa_outdim + p.sa2_outdim, p.sa2_outdim, p.fp3)
        self.fp2 = FPModule(p.sa2_outdim + p.sa1_outdim, p.sa1_outdim, p.fp2)
        self.fp1 = FPModule(p.sa1_outdim + in_channels, p.fp1_outdim, p.fp1)

        # Final linear layers at the output.
        self.lin1 = torch.nn.Linear(p.fp1_outdim, p.lin1_dim)
        self.lin2 = torch.nn.Linear(p.lin1_dim, p.lin2_dim)
        self.lin3 = torch.nn.Linear(p.lin2_dim, out_channels)
        self.out_act = p.out_act

    def forward(self, data: Data):
        sa0_out = (data.x, data.pos, data.batch)

        # Encode.
        sa1_out = self.sa1(*sa0_out)
        sa2_out = self.sa2(*sa1_out)
        sa3_out = self.sa3(*sa2_out)

        # Decode.
        fp3_out = self.fp3(*sa3_out, *sa2_out)
        fp2_out = self.fp2(*fp3_out, *sa1_out)
        x, _, _ = self.fp1(*fp2_out, *sa0_out)

        # Final layers.
        x = F.leaky_relu(self.lin1(x))
        x = F.leaky_relu(self.lin2(x))
        x = self.lin3(x)

        if self.out_act != "none":
            raise ValueError()

        return x
