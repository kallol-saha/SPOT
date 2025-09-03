"""Utilities for data collection for training dynamics model."""

from typing import Dict, List, Set, Tuple

import numpy as np
from numpy.typing import NDArray

from spot.pybullet_env.scene_gen.generate_scene import Scene

Array = NDArray[np.float32]


# The below are borrowed and modified from https://github.com/Learning-and-Intelligent-Systems/predicators/blob/88c2d21c103706cc1f8708eb5f0c0b2d0db51693/predicators/envs/blocks.py.
def _sample_initial_piles(
    env: Scene, rng: np.random.Generator, shuffle: bool = False
) -> List[List[int]]:
    """Returns a list of list of object names."""
    piles: List[List[int]] = []
    block_ids = env.grasp_obj_ids.copy()
    if shuffle:
        rng.shuffle(block_ids)
    for i, block_id in enumerate(block_ids):
        # If coin flip, start new pile
        if i == 0 or rng.uniform() < 0.2:
            piles.append([])
        # Add block to pile
        piles[-1].append(block_id)
    return piles


def _table_xy_is_clear(
    x: float,
    y: float,
    existing_xys: Set[Tuple[float, float]],
    collision_padding=2.0,
    block_size=0.05,
) -> bool:
    if all(
        abs(x - other_x) > collision_padding * block_size for other_x, _ in existing_xys
    ):
        return True
    if all(
        abs(y - other_y) > collision_padding * block_size for _, other_y in existing_xys
    ):
        return True
    return False


def _sample_initial_pile_xy(
    rng: np.random.Generator,
    existing_xys: Set[Tuple[float, float]],
    x_lb=-0.15,
    x_ub=0.15,
    y_lb=-0.15,
    y_ub=0.15,
) -> Tuple[float, float]:
    while True:
        x = rng.uniform(x_lb, x_ub)
        y = rng.uniform(y_lb, y_ub)
        if _table_xy_is_clear(x, y, existing_xys):
            return (x, y)


def _sample_state_from_piles(
    piles: List[List[int]],
    rng: np.random.Generator,
    table_height=0.65,
    block_size=0.05,
) -> Dict[str, Array]:
    data: Dict[str, Array] = {}
    # Create objects
    block_to_pile_idx = {}
    for i, pile in enumerate(piles):
        for j, block_id in enumerate(pile):
            assert block_id not in block_to_pile_idx
            block_to_pile_idx[block_id] = (i, j)
    # Sample pile (x, y)s
    pile_to_xy: Dict[int, Tuple[float, float]] = {}
    for i in range(len(piles)):
        pile_to_xy[i] = _sample_initial_pile_xy(rng, set(pile_to_xy.values()))

    # Create block states
    for block_id, pile_idx in block_to_pile_idx.items():
        pile_i, pile_j = pile_idx
        x_pos, y_pos = pile_to_xy[pile_i]
        z_pos = table_height + block_size * pile_j
        theta = np.random.uniform(0, 2 * np.pi)
        w, x, y, z = np.cos(theta / 2), 0.0, 0.0, np.sin(theta / 2)

        # [pos_x, pos_y, pos_z, x, y, z, w]
        data[block_id] = np.array([x_pos, y_pos, z_pos, x, y, z, w])

    return data
