from dataclasses import dataclass
from typing import Sequence

from torch import nn as nn


@dataclass
class MLPParams:
    # Hidden layer sizes.
    hidden: Sequence[int]

    # Whether to use batch norm.
    batch_norm: bool = True


def MLP(in_dim: int, out_dim: int, p: MLPParams):
    """Create a multilayer perceptron module. Takes in an input of shape: [BATCH x in_dim]
    and outputs a tensor of shape [BATCH, out_dim]. Internal architecture is constructed based on
    the parameters described in p.

    Args:
        in_dim: The input dimension.
        out_dim: The output dimension.
        p: parameters for creating the MLP.

    Returns:
        an MLP.
    """
    batch_norm = p.batch_norm
    channels = [in_dim, *p.hidden, out_dim]

    layers = []
    for i in range(1, len(channels)):
        layer_nodes = [nn.Linear(channels[i - 1], channels[i]), nn.ReLU()]
        if batch_norm:
            layer_nodes.append(nn.BatchNorm1d(channels[i]))
        layers.append(nn.Sequential(*layer_nodes))

    return nn.Sequential(*layers)
