#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Pulled from DCP

import os
import sys
import copy
import math
from tkinter import FALSE
import torch
import torch.nn as nn
import torch.nn.functional as F

# Part of the code is referred from: http://nlp.seas.harvard.edu/2018/04/03/attention.html#positional-encoding
from placement_suggester.models.multimodal_transformer_flow import DGCNN
from placement_suggester.models.multilaterate import MultilaterationHead
from placement_suggester.models.pointnet2 import PointNet2SSG, PointNet2MSG
from placement_suggester.models.pointnet2pyg import PN2DenseWrapper, PN2DenseParams 
from placement_suggester.models.vn_dgcnn import VN_DGCNN, VNArgs


class EquivariantFeatureEmbeddingNetwork(nn.Module):
    def __init__(self, emb_dims=512, emb_nn='dgcnn', input_dims=3, conditioning_size=0, last_relu=True):
        super(EquivariantFeatureEmbeddingNetwork, self).__init__()
        self.emb_dims = emb_dims
        self.emb_nn_name = emb_nn
        self.input_dims = input_dims
        self.conditioning_size = conditioning_size
        
        if emb_nn == 'dgcnn':
            self.emb_nn = DGCNN(
                emb_dims=self.emb_dims,
                input_dims=self.input_dims, 
                conditioning_size=self.conditioning_size,
                last_relu=last_relu
            )
        else:
            raise Exception('Not implemented')

    def forward(self, *input):
        points = input[0]  # B, 3, num_points
        points_dmean = points - \
            points.mean(dim=2, keepdim=True)
    
        points_embedding = self.emb_nn(
            points_dmean)  # B, emb_dims, num_points

        return points_embedding