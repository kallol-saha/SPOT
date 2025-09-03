<div align="center">

# Planning from Point Clouds over Continuous Actions for Multi-object Rearrangement
### CoRL 2025 (Oral Presentation - top 5.7%)

[Kallol Saha](https://kallol-saha.github.io/)<sup>&ast;1</sup>, [Amber Li](https://amburger66.github.io/)<sup>&ast;1</sup>, [Angela Rodriguez-Izquierdo](https://www.linkedin.com/in/angela-rodriguez-izq/)<sup>&ast;2</sup>, [Lifan Yu](https://www.linkedin.com/in/yu-lifan/)<sup>1</sup>, [Ben Eisner](https://beisner.me/)<sup>1</sup>, [Maxim Likhachev](https://www.cs.cmu.edu/~maxim/)<sup>1</sup>, [David Held](https://davheld.github.io/)<sup>1</sup><br>
<sup>&ast;</sup>Equal Contribution<br>
<sup>1</sup>Robotics Institute, Carnegie Mellon University &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; <sup>2</sup>Princeton University

</div>

<p align="center">
    <a href='https://arxiv.org/abs/2503.03045'>
      <img src='https://img.shields.io/badge/Paper-arXiv-green?style=plastic&logo=arXiv&logoColor=green' alt='Paper arXiv'>
    </a>
    <a href='https://planning-from-point-clouds.github.io/'>
      <img src='https://img.shields.io/badge/Project-Page-blue?style=plastic&logo=Google%20chrome&logoColor=blue' alt='Project Page'>
    </a>
</p>

<p align="center">
  <img src="SPOT_short_teaser.gif" width="80%"/>
</p>



We present **S**earch over **P**oint Cloud **O**bject **T**ransformations (**SPOT**), a method that solves long-horizon planning tasks such as multi-object rearrangement, by searching for a sequence of transformations from an initial scene point cloud to a goal-satisfying point cloud. SPOT uses guided A* search, expanding nodes by sampling actions from learned suggesters that operate on partially observed point clouds, thus eliminating the need to discretize actions or object relationships.

---
<br>

This is the official repository of [SPOT](https://planning-from-point-clouds.github.io/). If you find our work useful, please consider citing our paper:
```
@inproceedings{saha2025planning,
    title={Planning from Point Clouds over Continuous Actions for Multi-object Rearrangement},
    author={Saha, Kallol and Li, Amber and Rodriguez-Izquierdo, Angela and Yu, Lifan and Eisner, Ben and Likhachev, Maxim and Held, David},
    booktitle={Conference on Robot Learning (CoRL)},
    year={2025}
}
```
If you find any bugs in the code, or have any questions, feel free to raise an issue.

# Table of Contents

- [Installation](#installation)
   * [Clone the repository and submodules](#clone-the-repository-and-submodules)
   * [Graph Visualization](#graph-visualization)
   * [Install SPOT](#install-spot)
- [Quick Demo](#quick-demo)
- [Running our Method](#running-our-method)
   * [1. Collecting Data](#1-collecting-data)
   * [2. Training Suggesters](#2-training-suggesters)
      * [Training the placement suggester](#training-the-placement-suggester)
      * [Training the object suggester](#training-the-object-suggester)
      * [Training the Model Deviation Estimator (MDE)](#training-the-model-deviation-estimator-mde)
   * [3. Planning](#3-planning)
   * [4. Execution](#4-execution)
      * [Simulation Execution](#simulation-execution)
      * [Real World Execution](#real-world-execution)

# Installation

### Clone the repository and submodules:
```bash
git clone --recursive https://github.com/kallol-saha/SPOT.git
cd SPOT
```

### Graph Visualization:
We need graphviz for visualizing graphs:
```bash
sudo apt-get install graphviz
```

If you cannot use sudo or do not want to visualize graphs, disable the global flag: (This is already set to FALSE by default)

```python
# In files that use graph visualization (e.g., `spot/evaluation.py`, `scripts/real_world/plan_real_world.py`)
VISUALIZE_GRAPH = False  # set to True to enable graph visualization
```

### Install SPOT:
We use Python 3.9 within a conda environment. We use PyTorch with CUDA 11.7. Use the bash script below to complete the full installation in one command:
```bash
chmod +x install_spot.sh
sh install_spot.sh
```

# Quick Demo:
Loads pre-trained models (placement suggester, object suggester, MDE), creates an A* planner, generates and evaluates a plan.

```bash
python demo.py
```

# Running our Method:

1. We collect human demonstration data in both simulation and real world environments.
2. We train 3 learned modules from human demonstration data: (a) Placement suggester, (b) Object suggester, and (c) Model Deviation Estimator (MDE)
3. Given a test dataset of initial point clouds, we compute plans (a sequence of transformations applied to objects) using guided A* search. 
4. To benchmark our method, we execute the plans found in simulation and real world environments.


## 1. Collecting Data:

For simulation data collection, we move objects with the keyboard and record the final transformations. 
```bash
python scripts/collect_keyboard_demos.py
```

For real world data collection, we extract 2D Tracks, from which we extract 2D transformations. See this [repo](https://github.com/kallol-saha/video_to_transforms)

## 2. Training Suggesters:

You can train each of the suggesters from scratch, or you can skip this step and use the pre-trained weights downloaded under assets and run planning and execution directly.

### Training the placement suggester
Trains an equivariant multi-modal transformer network ([TAXPose-D](https://github.com/kallol-saha/placement-suggester)) that suggests object placement poses given point cloud observations to learn placement distributions from demonstration data.
```bash
python scripts/training/train_placement_suggester.py
```

### Training the object suggester
Trains a PointNet++ classifier model that determines which objects should be moved next during planning:
```bash
python scripts/training/train_object_suggester.py
```

### Training the Model Deviation Estimator (MDE)
Trains a PointNet++ model that predicts how much the environment state differs from expected after executing actions. Supports both classification and regression modes, with datasets for blocks and table bussing environments.
```bash
python scripts/training/train_mde.py
```

## 3. Planning:
Evaluates the planning performance of SPOT across test datasets. Loads trained models, runs guided A* search-based planning on point clouds, and saves metrics including success rates, planning times, and node expansion counts.
```bash
python scripts/planning/benchmark_plans.py
```

## 4. Execution:

### Simulation Execution:
Executes the plans generated by the planner in a PyBullet simulation environment. Applies the planned object transformations sequentially using motion planning, records execution success/failure, and saves execution metrics to evaluate plan feasibility.
```bash
python scripts/planning/benchmark_sim_execution.py
```

### Real World Execution:

We use a Franka Panda for executing SPOT in a real world environment.<br>
`scripts/real_world/plan_real_world.py`: Generates real world plans<br>
`scripts/real_world/get_plans_from_goals.py`: Selects the best plan

