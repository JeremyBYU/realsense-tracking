# Docker file to setup base robotics environment
# Ubunut base, python 3, cmake 3.15, realsense, open cv, 
ARG FROM_IMAGE=ubuntu:bionic
FROM $FROM_IMAGE
ARG TARGETPLATFORM

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
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
    
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
RUN apt-get install --no-install-recommends -y graphviz build-essential zlib1g-dev libhdf5-dev libprotobuf-dev libprotoc-dev protobuf-compiler

# Install cmake - This takes like 30 minutes on emulated hardware...
RUN wget -O /opt/cmake-3.15.5.tar.gz https://cmake.org/files/v3.15/cmake-3.15.5.tar.gz && \
    cd /opt && \
    tar -xzf cmake-3.15.5.tar.gz && \
    rm /opt/cmake-3.15.5.tar.gz && \
    cd /opt/cmake-3.15.5 && \
    ./configure --prefix=/opt/cmake-3.15.5 && \
    make -j && \
    make install && \
    rm -rf Source && rm -rf Bootstrap.cmk && rm -rf Tests && rm -rf Utilities

ENV PATH "/opt/cmake-3.15.5/bin:$PATH"

# Prepare OpenCV Creation File
COPY opencv.sh /tmp/  
RUN chmod u+x /tmp/opencv.sh

# Clone OpenCV - Computer Vision
RUN git clone -b '3.4.7' --single-branch https://github.com/opencv/opencv.git /opt/opencv && \
    git clone -b '3.4.7' --single-branch https://github.com/opencv/opencv_contrib.git /opt/opencv_contrib && \
    bash /tmp/opencv.sh ${TARGETPLATFORM}

# Prepare for ECAL - Shared Memory Marshalling and Communication
RUN cd /opt && \
    git clone --recursive git://github.com/continental/ecal.git && \
    cd ecal && mkdir build_ &&  cd build_ && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DECAL_THIRDPARTY_BUILD_PROTOBUF=OFF -DHAS_QT5=OFF -DBUILD_DOCS=OFF -DBUILD_PY_BINDING=ON && \
    make -j && \
    cd /opt/ecal/build_ && \
    cpack -G DEB && \
    dpkg -i _deploy/eCAL-* && \
    cd /opt/ecal/build_ && cmake --build . --target create_python_egg --config Release && cd python && pip3 install . && \
    rm -rf /opt/ecal

# Clone Realsense - Depth, IMU, 6DOF Sensor SDK
RUN git clone https://github.com/IntelRealSense/librealsense.git /opt/librealsense && cd /opt/librealsense && git checkout development && \
    mkdir build && cd build && \
    cmake ../ -DFORCE_RSUSB_BACKEND=true -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true -DBUILD_PYTHON_BINDINGS=true && \
    make -j && \
    make install && \
    rm -rf /opt/librealsense

ENV PYTHONPATH $PYTHONPATH:/usr/local/lib

# Prepare XIVO - Visual SLAM on Host Computer
# COPY xivo.sh /tmp/  
# RUN chmod u+x /tmp/xivo.sh
# RUN bash /tmp/xivo.sh ${TARGETPLATFORM} 

# Link python to python3
RUN ln -s /usr/bin/python3 /usr/bin/python & \
    ln -s /usr/bin/pip3 /usr/bin/pip

# Open 3D Install
RUN cd /opt && git clone --recursive https://github.com/intel-isl/Open3D && \
    cd /opt/Open3D && \
    mkdir build && cd build && \
    cmake ../ -DBUILD_CPP_EXAMPLES=OFF -DENABLE_JUPYTER=OFF -DCMAKE_CXX_STANDARD_LIBRARIES="-lX11 -ldl" && \
    make -j && \
    make install && make install-pip-package && \
    rm -rf /opt/Open3D

# Stuff I like/need
RUN apt-get install --no-install-recommends -y nano htop

# Our Project Dependencies
RUN apt-get install --no-install-recommends -y libgflags-dev libgoogle-glog-dev python3-scipy python3-matplotlib
RUN pip3 install protobuf

# setup entrypoint
COPY ./base_entrypoint.sh /

# TO ADD LATER

ENTRYPOINT ["/base_entrypoint.sh"]
CMD ["bash"]