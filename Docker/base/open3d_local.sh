#!/bin/zsh

# exit when any command fails
set -e

# keep track of the last executed command
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
# echo an error message before exiting
trap 'echo "\"${last_command}\" command filed with exit code $?."' EXIT


# Clone Open3D
git clone --single-branch --branch v0.12.0 --recursive https://github.com/intel-isl/Open3D.git ~/Software/Open3D

cd ~/Software/Open3D
mkdir build
cd build
cmake .. -D CMAKE_BUILD_TYPE=release \
    -D BUILD_CPP_EXAMPLES=OFF \
    -D ENABLE_JUPYTER=OFF \
    -D GLIBCXX_USE_CXX11_ABI=ON \
    -D USE_SYSTEM_EIGEN3=ON


make -j12
sudo make install
make install-pip-package