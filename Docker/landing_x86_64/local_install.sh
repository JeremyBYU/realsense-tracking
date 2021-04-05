#!/bin/zsh
# exit when any command fails
set -e

# keep track of the last executed command
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
# echo an error message before exiting
trap 'echo "\"${last_command}\" command filed with exit code $?."' EXIT

INSTALL_DIR=/home/jeremy/Software

# Clone Polylidar3D
git clone -b 'landing' --single-branch --recursive https://github.com/JeremyBYU/polylidar.git $INSTALL_DIR/polylidar && \
cd $INSTALL_DIR/polylidar && mkdir cmake-build && cd cmake-build && \
cmake .. -DCMAKE_BUILD_TYPE=Release -DFETCHCONTENT_QUIET=OFF && \
make -j${N_CPUS} && \
cmake --build . --target python-package --config Release -j${N_CPUS} && \
cd lib/python_package && pip install .

# Clone FastGaussianAccumulator
git clone -b 's2beta' --single-branch --recursive https://github.com/JeremyBYU/FastGaussianAccumulator.git $INSTALL_DIR/fastga && \
cd $INSTALL_DIR/fastga && mkdir cmake-build && cd cmake-build && \
cmake .. -DCMAKE_BUILD_TYPE=Release -DFETCHCONTENT_QUIET=OFF && \
make -j${N_CPUS} && \
cmake --build . --target python-package --config Release -j${N_CPUS} && \
cd lib/python_package && pip install .

# Clone OrganizedPointFilters
git clone --recursive https://github.com/JeremyBYU/OrganizedPointFilters.git $INSTALL_DIR/opf && \
cd $INSTALL_DIR/opf && mkdir cmake-build && cd cmake-build && \
cmake .. -DCMAKE_BUILD_TYPE=Release -DFETCHCONTENT_QUIET=OFF && \
make -j${N_CPUS} && \
cmake --build . --target python-package --config Release -j${N_CPUS} && \
cd lib/python_package && pip install .


git clone --recursive https://github.com/JeremyBYU/polylabelfast.git $INSTALL_DIR/polylabelfast && \
cd $INSTALL_DIR/polylabelfast && \
pip install .