#!/bin/bash
# exit when any command fails
set -e

# keep track of the last executed command
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
# echo an error message before exiting
trap 'echo "\"${last_command}\" command filed with exit code $?."' EXIT

INSTALL_DIR=$HOME/Software
cmake_arch=""

unameOut="$(uname -s)"
case "${unameOut}" in
    Linux*)     machine=Linux;;
    Darwin*)    machine=Mac;;
    CYGWIN*)    machine=Cygwin;;
    MINGW*)     machine=MinGw;;
    MSYS_NT*)   
                machine=Windows;
                cmake_arch="-DCMAKE_GENERATOR_PLATFORM=x64"
                ;;
    *)          machine="UNKNOWN:${unameOut}"
esac
echo "${machine} and ${cmake_arch}"

# Clone Polylidar3D
git clone -b 'landing' --single-branch --recursive https://github.com/JeremyBYU/polylidar.git $INSTALL_DIR/polylidar && \
cd $INSTALL_DIR/polylidar && mkdir cmake-build && cd cmake-build && \
cmake .. -DCMAKE_BUILD_TYPE=Release -DFETCHCONTENT_QUIET=OFF ${cmake_arch} && \
cmake --build -j${N_CPUS} . && \
cmake --build . --target python-package --config Release -j${N_CPUS} && \
cd lib/python_package && pip install .

# Clone FastGaussianAccumulator
git clone -b 's2beta' --single-branch --recursive https://github.com/JeremyBYU/FastGaussianAccumulator.git $INSTALL_DIR/fastga && \
cd $INSTALL_DIR/fastga && mkdir cmake-build && cd cmake-build && \
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O3 -DNDEBUG -msse4.2" -DFETCHCONTENT_QUIET=OFF ${cmake_arch} && \
cmake --build -j${N_CPUS} . && \
cmake --build . --target python-package --config Release -j${N_CPUS} && \
cd lib/python_package && pip install .

# Clone OrganizedPointFilters
git clone --recursive https://github.com/JeremyBYU/OrganizedPointFilters.git $INSTALL_DIR/opf && \
cd $INSTALL_DIR/opf && mkdir cmake-build && cd cmake-build && \
cmake .. -DCMAKE_BUILD_TYPE=Release -DFETCHCONTENT_QUIET=OFF ${cmake_arch} && \
cmake --build -j${N_CPUS} . && \
cmake --build . --target python-package --config Release -j${N_CPUS} && \
cd lib/python_package && pip install .

# Polylabel
git clone --recursive https://github.com/JeremyBYU/polylabelfast.git $INSTALL_DIR/polylabelfast && \
cd $INSTALL_DIR/polylabelfast && \
pip install .