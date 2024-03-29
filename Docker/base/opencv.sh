#!/bin/bash
# IS_ARM64=$(( "$1" == "linux/arm64" ? 1 : 0 ))
set -e
CMAKE_ARGS_ENV="" && [[ "$1" == "linux/arm64" ]]  && CMAKE_ARGS_ENV="-latomic"

# CMAKE_ARGS_ENV=${IS_ARM64:+-latomic}
echo $CMAKE_ARGS_ENV

cd /opt/opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D OPENCV_EXTRA_MODULES_PATH=/opt/opencv_contrib/modules \
    -D BUILD_TESTS=OFF \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D OPENCV_ENABLE_NONFREE=ON \
    -D PYTHON_EXECUTABLE=/usr/bin/python3 \
    -D ENABLE_CXX11=ON \
    -D CMAKE_SHARED_LINKER_FLAGS=$CMAKE_ARGS_ENV \
    -D ENABLE_PRECOMPILED_HEADERS=OFF \
    -D BUILD_EXAMPLES=OFF ../
# -D ENABLE_NEON=ON \
# -D ENABLE_VFPV3=ON \
# https://github.com/opencv/opencv/issues/14868
make -j${N_CPUS}
make install
rm -rf /opt/opencv_contrib
rm -rf /opt/opencv