#!/bin/bash

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