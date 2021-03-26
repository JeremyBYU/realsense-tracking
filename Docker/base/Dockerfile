# Docker file to setup base robotics environment
# Ubunut base, python 3, cmake 3.15, realsense, open cv, 
ARG FROM_IMAGE=ubuntu:bionic
FROM $FROM_IMAGE as base_install
ARG TARGETPLATFORM
ARG N_CPUS=12

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && apt-get install -q -y tzdata && rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y \
    bash-completion \
    lsb-release \
    python3-pip \
    python3-numpy  \
    python3-dev \
    git \
    libc++-7-dev \
    libc++abi-7-dev \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

RUN echo ${N_CPUS}

# Link python to python3
RUN ln -s /usr/bin/python3 /usr/bin/python & \
    ln -s /usr/bin/pip3 /usr/bin/pip
    
# clone source
ENV MYWS /opt/workspace
WORKDIR $MYWS

# Realsense Dependencies
RUN apt-get update && \
    apt-get install -q -y wget git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev && \
    apt-get install -q -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

# OpenCV Dependencies
RUN apt-get install --no-install-recommends -y build-essential libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev \
                                               libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev \
                                               libatlas-base-dev libcanberra-gtk* \
                                               libglew-dev liblz4-dev
# ECAL Dependencies
RUN apt-get install --no-install-recommends -y graphviz build-essential zlib1g-dev libhdf5-dev libprotobuf-dev libprotoc-dev protobuf-compiler libcurl4-openssl-dev

# Open3D Dependencies
RUN apt-get install --no-install-recommends -y libeigen3-dev libblas-dev liblapack-dev liblapacke-dev libxi-dev

FROM base_install as cmake_install

# Install cmake - This takes like 30 minutes on emulated hardware...
RUN wget -O /opt/cmake-3.18.6.tar.gz https://cmake.org/files/v3.18/cmake-3.18.6.tar.gz && \
    cd /opt && \
    tar -xzf cmake-3.18.6.tar.gz && \
    rm /opt/cmake-3.18.6.tar.gz && \
    cd /opt/cmake-3.18.6 && \
    ./configure --prefix=/opt/cmake-3.18.6 && \
    make -j${N_CPUS} && \
    make install && \
    rm -rf Source && rm -rf Bootstrap.cmk && rm -rf Tests && rm -rf Utilities

ENV PATH "/opt/cmake-3.18.6/bin:$PATH"

FROM cmake_install as opencv_install
# Prepare OpenCV Creation File
COPY opencv.sh /tmp/  
RUN chmod u+x /tmp/opencv.sh

# Clone OpenCV - Computer Vision
RUN git clone -b '3.4.7' --single-branch https://github.com/opencv/opencv.git /opt/opencv && \
    git clone -b '3.4.7' --single-branch https://github.com/opencv/opencv_contrib.git /opt/opencv_contrib && \
    bash /tmp/opencv.sh ${TARGETPLATFORM}

FROM opencv_install as ecal_install
# Prepare for ECAL - Shared Memory Marshalling and Communication
RUN cd /opt && \
    git clone --single-branch --branch v5.8.5 --recursive git://github.com/continental/ecal.git
RUN cd /opt/ecal && mkdir build_ &&  cd build_ && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DECAL_THIRDPARTY_BUILD_PROTOBUF=OFF -DHAS_QT5=OFF -DBUILD_DOCS=OFF -DECAL_THIRDPARTY_BUILD_CURL=OFF -DECAL_THIRDPARTY_BUILD_HDF5=OFF -DECAL_THIRDPARTY_BUILD_TINYXML2=ON -DBUILD_PY_BINDING=ON && \
    make -j${N_CPUS} && \
    cd /opt/ecal/build_ && \
    cpack -G DEB && \
    dpkg -i _deploy/eCAL-* && \
    cd /opt/ecal/build_ && cmake --build . --target create_python_wheel --config Release && pip install _deploy/ecal-* && \
    rm -rf /opt/ecal

FROM ecal_install as realsense_install
# Clone Realsense - Depth, IMU, 6DOF Sensor SDK
RUN git clone --single-branch --branch v2.40.0 --recursive https://github.com/IntelRealSense/librealsense.git /opt/librealsense && \
    cd /opt/librealsense && mkdir build && cd build && \
    cmake ../ -DFORCE_RSUSB_BACKEND=true -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true -DBUILD_PYTHON_BINDINGS=true && \
    make -j12 && \
    make install && \
    echo "from .pyrealsense2 import *" >> /usr/lib/python3/dist-packages/pyrealsense2/__init__.py && \
    rm -rf /opt/librealsense

ENV PYTHONPATH $PYTHONPATH:/usr/local/lib

FROM realsense_install as o3d_install
# Open 3D Install
COPY o3d.patch /opt/o3d.patch
RUN git clone --single-branch --branch v0.12.0 --recursive https://github.com/intel-isl/Open3D.git /opt/Open3D && \
    cd /opt/Open3D && git apply ../o3d.patch && \
    mkdir build && cd build && \
    cmake ../ -DBUILD_CPP_EXAMPLES=OFF -DBUILD_EXAMPLES=OFF -DBUILD_JUPYTER_EXTENSION=OFF -DENABLE_JUPYTER=OFF -DGLIBCXX_USE_CXX11_ABI=ON -DUSE_SYSTEM_EIGEN3=ON -DBUILD_GUI=OFF -DBUILD_FILAMENT_FROM_SOURCE=OFF && \
    make -j12 && \
    make install && make install-pip-package && \
    rm -rf /opt/Open3D

FROM o3d_install as final_install
# Stuff I like/need
RUN apt-get install --no-install-recommends -y nano htop

# Our Project Dependencies
RUN apt-get install --no-install-recommends -y libgflags-dev libgoogle-glog-dev python3-scipy python3-matplotlib
RUN pip3 install protobuf

COPY ecal.ini /usr/local/etc/ecal/ecal.ini
# setup entrypoint
COPY ./base_entrypoint.sh /

# TO ADD LATER
ENTRYPOINT ["/base_entrypoint.sh"]
CMD ["bash"]