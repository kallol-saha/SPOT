#!/bin/bash

set -euo pipefail

DATASETS_DIR=src/spot/pybullet_env/
mkdir -p $DATASETS_DIR
ZIPFILE_NAME=data.zip
ZIPFILE_PATH=$DATASETS_DIR/$ZIPFILE_NAME
FILE_ID=1MaxTUQXfRLIBLzZSfzqqsMrTg9YZs75t


echo "Downloading $ZIPFILE_NAME"
wget --load-cookies /tmp/cookies.txt "https://docs.google.com/uc?export=download&confirm=$(wget --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate "https://docs.google.com/uc?export=download&id=$FILE_ID" -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')&id=$FILE_ID" -O $ZIPFILE_PATH && rm -rf /tmp/cookies.txt
echo "Downloaded $ZIPFILE_NAME"

# Unzip the file
echo "Unzipping $ZIPFILE_NAME"
unzip -q $ZIPFILE_PATH -d $DATASETS_DIR
echo "Unzipped $ZIPFILE_NAME"

# Remove the zip file
rm $ZIPFILE_PATH
