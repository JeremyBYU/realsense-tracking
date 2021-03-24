#!/bin/zsh

# exit when any command fails
set -e

# keep track of the last executed command
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
# echo an error message before exiting
trap 'echo "\"${last_command}\" command filed with exit code $?."' EXIT

# Clone Realsense
git clone --single-branch --branch v2.40.0 --recursive https://github.com/IntelRealSense/librealsense.git ~/Software/realsense

cd ~/Software/realsense
mkdir build
cd build
cmake .. -D CMAKE_BUILD_TYPE=release \
    -D FORCE_RSUSB_BACKEND=true \
    -D BUILD_EXAMPLES=true \
    -D BUILD_GRAPHICAL_EXAMPLES=true \
    -D BUILD_PYTHON_BINDINGS=true


make -j12
sudo make install
# https://github.com/IntelRealSense/librealsense/issues/6820#issuecomment-660167748
sudo chown jeremy -R ~/miniconda3/envs/realsense/lib/python3.8/site-packages/pyrealsense2 
echo "from .pyrealsense2 import *"  >> ~/miniconda3/envs/realsense/lib/python3.8/site-packages/pyrealsense2/__init__.py


