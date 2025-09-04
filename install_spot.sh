#!/bin/bash

echo ""
echo "------------------- Creating Conda Environment -----------------"
echo ""

# Environment creation
source $HOME/miniconda3/etc/profile.d/conda.sh
conda create -n spot python=3.9
conda activate spot

echo ""
echo "------------------- Setting up CUDA 11.7 -----------------"
echo ""

# Install cuda 11.7 (required by pytorch3d, torch cluster and torch geometric)
conda install -c nvidia/label/cuda-11.7.0 cuda-toolkit

# Dynamically detect the active environment prefix (works for conda/mamba)
ENV_PREFIX=${CONDA_PREFIX:-$(python3 -c 'import sys; print(sys.prefix)')}

# Export variables for the current shell session
export CUDA_HOME="$ENV_PREFIX"
export PATH="$ENV_PREFIX/bin:$PATH"
export LD_LIBRARY_PATH="$ENV_PREFIX/lib:$LD_LIBRARY_PATH"
export CPATH="$ENV_PREFIX/include:$CPATH"

# Persist these settings to ~/.bashrc in an idempotent and dynamic way
cat >> "$HOME/.bashrc" <<'EOF'
# >>> spot env paths >>>
# Automatically set CUDA and library paths based on the active conda environment
if [[ -n "$CONDA_PREFIX" ]]; then
    export CUDA_HOME="$CONDA_PREFIX"
    export PATH="$CONDA_PREFIX/bin:$PATH"
    export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$LD_LIBRARY_PATH"
    export CPATH="$CONDA_PREFIX/include:$CPATH"
fi
# <<< spot env paths <<<
EOF

source $HOME/.bashrc
conda activate spot

echo ""
echo "------------------- Install Packages -----------------"
echo ""

# Pytorch3D Installation:
conda install -c fvcore -c iopath -c conda-forge fvcore iopath
conda install -c bottler nvidiacub
conda install pytorch3d=0.7.7=py39_cu117_pyt201 -c pytorch3d

# Other Conda dependencies:
conda install -c conda-forge libstdcxx-ng       # For compiling ikfast
conda install fsspec
conda install pytz=2024.1

# Install pip packages:
pip install -r requirements.txt

echo ""
echo "------------------- Install Torch -----------------"
echo ""
conda install pytorch==2.0.1 torchvision==0.15.2 pytorch-cuda=11.7 -c pytorch -c nvidia

echo ""
echo "------------------- Installing Submodules -----------------"
echo ""

cd spot/submodules/pyg_libs
pip install -e .
cd ..
cd placement_suggester
pip install -e .
cd ..
cd pybullet_planning
sh setup_ikfast.sh
cd ../../..

echo ""
echo "------------------- Installing SPOT -----------------"
echo ""

# Main project installation:
pip install -e .

echo ""
echo "------------------- Installing Assets -----------------"
echo ""

# Downloading assets:
gdown 1ch1ezvY2x-bMpgBzBTsC1ZzFDKkutiPT
unzip assets.zip -d ./
rm -rf assets.zip 

cd assets/eval/
# Unzip all zip files in eval directory and remove them
for f in *.zip; do
    unzip "$f"
    rm "$f"
done