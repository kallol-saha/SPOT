from dataclasses import asdict, dataclass
from typing import Dict

import torch
import torch.nn as nn
import torch_geometric.nn as tgnn
from torch_geometric.data import Batch, Data

from rpad.pyg.nets.mlp import MLP, MLPParams


@dataclass
class DynamicEdgeConvParams:
    mlp_params: MLPParams
    k: int = 30
    aggr: str = "max"

    def asdict(self) -> Dict[str, str]:
        return {k: v for k, v in asdict(self).items() if k != "mlp_params"}


@dataclass
class DGCNNDenseParams:
    c1: DynamicEdgeConvParams = DynamicEdgeConvParams(MLPParams((64,)))
    c1_outdim: int = 64

    c2: DynamicEdgeConvParams = DynamicEdgeConvParams(MLPParams((64,)))
    c2_outdim: int = 64

    c3: DynamicEdgeConvParams = DynamicEdgeConvParams(MLPParams((64,)))
    c3_outdim: int = 64

    mlp: MLPParams = MLPParams((1024, 256), batch_norm=False)
    final_layer_dim: int = 128


class DGCNNDense(nn.Module):
    def __init__(
        self,
        in_chan: int = 0,
        out_chan: int = 3,
        p: DGCNNDenseParams = DGCNNDenseParams(),
    ):
        """This is the dense prediction version of DGCNN from their original paper.

        Args:
            in_chan: The number of additional feature channels at input. i.e. mask => in_chan=1
            out_chan: The dimensionality of per-point prediction.
            p: DGCNN hyperparameter.
        """
        super().__init__()

        # Inside DynamicEdgeConv, they need an MLP which doubles things.
        c1_indim = (in_chan + 3) * 2
        self.c1 = tgnn.DynamicEdgeConv(
            MLP(c1_indim, p.c1_outdim, p.c1.mlp_params), **p.c1.asdict()
        )
        c2_indim = p.c1_outdim * 2
        self.c2 = tgnn.DynamicEdgeConv(
            MLP(c2_indim, p.c2_outdim, p.c2.mlp_params), **p.c2.asdict()
        )
        c3_indim = p.c2_outdim * 2
        self.c3 = tgnn.DynamicEdgeConv(
            MLP(c3_indim, p.c3_outdim, p.c3.mlp_params), **p.c3.asdict()
        )

        # We're gonna concat everything.
        self.mlp = MLP(
            p.c1_outdim + p.c2_outdim + p.c3_outdim, p.final_layer_dim, p.mlp
        )
        self.final_layer = nn.Linear(p.final_layer_dim, out_chan)

    def forward(self, data: Data):
        x, pos, batch = data.x, data.pos, data.batch
        if x is not None:
            x = torch.cat([x, pos], dim=-1)
        else:
            x = pos

        x1 = self.c1(x, batch)
        x2 = self.c2(x1, batch)
        x3 = self.c3(x2, batch)

        x = self.mlp(torch.cat([x1, x2, x3], dim=-1))
        x = self.final_layer(x)
        return x


if __name__ == "__main__":
    net = DGCNNDense()
    data = Data(pos=torch.rand((1000, 3)))
    batch = Batch.from_data_list([data])
    res = net(batch)
