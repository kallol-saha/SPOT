import numpy as np
import torch

def to_np(x):
    """!@brief Converts tensor or variable x to numpy array

    Returns numpy array of pytorch object.

    @param x Tensor or variable.
    """
    if(isinstance(x, np.ndarray)):
        return x
    elif torch.is_tensor(x):
        return x.detach().cpu().numpy()
    else:
        return x.detach().data.cpu().numpy()
