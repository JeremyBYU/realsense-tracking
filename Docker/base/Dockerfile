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

# Prepare realsense
RUN apt-get update && \
    apt-get install -q -y wget git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev && \
    apt-get install -q -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

# Install cmake - This takes like 30 minutes on emulated hardware...
RUN wget -O /opt/cmake-3.15.5.tar.gz https://cmake.org/files/v3.15/cmake-3.15.5.tar.gz && \
    cd /opt && \
    tar -xzf cmake-3.15.5.tar.gz && \
    rm /opt/cmake-3.15.5.tar.gz && \
    cd /opt/cmake-3.15.5 && \
    ./configure --prefix=/opt/cmake-3.15.5 && \
    make -j18 && \
    make install

# # To save time we are just copying the binary cmake executables that are already compiled
# COPY bin/cmake_bin/cmake-3.15.5.tar.gz /opt/cmake-3.15.5.tar.gz
# RUN tar -xzvf /opt/cmake-3.15.5.tar.gz -C /opt/ && rm -rf /opt/cmake-3.15.5.tar.gz
ENV PATH "/opt/cmake-3.15.5/bin:$PATH"

# Clone Realsense - Depth and IMU Sensor SDK
RUN git clone https://github.com/IntelRealSense/librealsense.git /opt/librealsense && cd /opt/librealsense && \
    mkdir build && cd build && \
    cmake ../ -DFORCE_RSUSB_BACKEND=true -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true -DBUILD_PYTHON_BINDINGS=true && \
    make -j 18 && \
    make install

ENV PYTHONPATH $PYTHONPATH:/usr/local/lib


# Clone OpenCV - Computer Vision
RUN git clone -b '3.4.7' --single-branch https://github.com/opencv/opencv.git /opt/opencv
RUN git clone -b '3.4.7' --single-branch https://github.com/opencv/opencv_contrib.git /opt/opencv_contrib

RUN apt-get install --no-install-recommends -y build-essential libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
RUN apt-get install --no-install-recommends -y libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev
RUN apt-get install --no-install-recommends -y libatlas-base-dev libcanberra-gtk*

COPY opencv.sh /tmp/  
RUN chmod u+x /tmp/opencv.sh
RUN bash /tmp/opencv.sh ${TARGETPLATFORM}

RUN apt-get install --no-install-recommends -y libglew-dev liblz4-dev

# Prepare for ECAL - Shareed Memory Marshalling and Communication
RUN apt-get install --no-install-recommends -y git doxygen graphviz build-essential zlib1g-dev libhdf5-dev libprotobuf-dev libprotoc-dev protobuf-compiler
RUN cd /opt && \
    git clone --recursive git://github.com/continental/ecal.git && \
    cd ecal && mkdir build_ &&  cd build_ && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DECAL_THIRDPARTY_BUILD_PROTOBUF=OFF -DHAS_QT5=OFF && \
    make -j16

RUN cd /opt/ecal/build_ && \
    cpack -G DEB && \
    dpkg -i _deploy/eCAL-*

# Prepare XIVO - Visual SLAM
# COPY xivo.sh /tmp/  
# RUN chmod u+x /tmp/xivo.sh
# RUN bash /tmp/xivo.sh ${TARGETPLATFORM} 

# Stuff I like/need
RUN apt-get install --no-install-recommends -y nano htop

# Our Project Dependencies
RUN apt-get install --no-install-recommends -y libgflags-dev

# setup entrypoint
COPY ./base_entrypoint.sh /

ENTRYPOINT ["/base_entrypoint.sh"]
CMD ["bash"]