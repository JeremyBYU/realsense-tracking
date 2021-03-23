#!/bin/zsh

# Clone Open CV
git clone --single-branch --branch v5.8.5 --recursive git://github.com/continental/ecal.git ~/Software/ecal

cd ~/Software/ecal
mkdir build_
cd build_
cmake .. -D CMAKE_BUILD_TYPE=Release \
    -D ECAL_THIRDPARTY_BUILD_PROTOBUF=OFF \
    -D ECAL_THIRDPARTY_BUILD_CURL=OFF \
    -D ECAL_THIRDPARTY_BUILD_HDF5=OFF \
    -D HAS_QT5=ON \
    -D BUILD_DOCS=OFF \
    -D ECAL_THIRDPARTY_BUILD_TINYXML2=ON \
    -D BUILD_PY_BINDING=ON


make -j12
cpack -G DEB
sudo dpkg -i _deploy/eCAL-*
cmake --build . --target create_python_wheel --config Release
# activate conda and do this shit
pip install _deploy/ecal-*