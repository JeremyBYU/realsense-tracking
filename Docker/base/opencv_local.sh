#!/bin/zsh

# exit when any command fails
set -e

# keep track of the last executed command
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
# echo an error message before exiting
trap 'echo "\"${last_command}\" command filed with exit code $?."' EXIT

# Clone Open CV
git clone -b '3.4.7' --single-branch https://github.com/opencv/opencv.git ~/Software/opencv
git clone -b '3.4.7' --single-branch https://github.com/opencv/opencv_contrib.git ~/Software/opencv_contrib

cd ~/Software/opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D OPENCV_EXTRA_MODULES_PATH=~/Software/opencv_contrib/modules \
    -D BUILD_TESTS=OFF \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D OPENCV_ENABLE_NONFREE=ON \
    -D PYTHON_EXECUTABLE=/usr/bin/python3 \
    -D ENABLE_CXX11=ON \
    -D ENABLE_PRECOMPILED_HEADERS=OFF \
    -D BUILD_EXAMPLES=OFF ../


make -j
make install
# rm -rf ~/Software/opencv_contrib