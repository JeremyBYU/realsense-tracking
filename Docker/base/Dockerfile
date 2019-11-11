# This is an auto generated Dockerfile for ros2:devel
# generated from docker_images_ros2/devel/create_ros_image.Dockerfile.em
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
    dirmngr \
    gnupg2 \
    lsb-release \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros2/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros2-latest.list

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    git \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# install python packages
RUN pip3 install -U \
    argcomplete \
    colcon-common-extensions \
    colcon-mixin \
    flake8 \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures

# FIXME This is a workaround for pytest not found causing builds to fail
# Following RUN statements tests for regression of https://github.com/ros2/ros2/issues/722
RUN pip3 freeze | grep pytest \
    && python3 -m pytest --version

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# clone source
ENV ROS2_WS /opt/ros2_ws
RUN mkdir -p $ROS2_WS/src
WORKDIR $ROS2_WS

# build source
RUN colcon \
    build \
    --cmake-args \
      -DSECURITY=ON --no-warn-unused-cli \
    --symlink-install


# Prepare realsense
RUN sudo apt-get update && \
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

# Clone Realsense
RUN git clone https://github.com/IntelRealSense/librealsense.git /opt/librealsense && cd /opt/librealsense && \
    mkdir build && cd build && \
    cmake ../ -DFORCE_RSUSB_BACKEND=true -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true -DBUILD_PYTHON_BINDINGS=true && \
    make -j 18 && \
    make install


# Clone OpenCV
RUN git clone -b '3.4.7' --single-branch https://github.com/opencv/opencv.git /opt/opencv
RUN git clone -b '3.4.7' --single-branch https://github.com/opencv/opencv_contrib.git /opt/opencv_contrib

RUN sudo apt-get install --no-install-recommends -y build-essential libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
RUN sudo apt-get install --no-install-recommends -y libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev
RUN sudo apt-get install --no-install-recommends -y python3-numpy 

COPY opencv.sh /tmp/  
RUN chmod u+x /tmp/opencv.sh
RUN bash /tmp/opencv.sh ${TARGETPLATFORM} 

# install packages
RUN apt-get update && apt-get install -q -y \
    libasio-dev \
    libtinyxml2-dev \
    wget \
    && rm -rf /var/lib/apt/lists/*

ARG ROS_DISTRO=dashing
ENV ROS_DISTRO=$ROS_DISTRO
ENV ROS_VERSION=2 \
    ROS_PYTHON_VERSION=3


RUN wget https://raw.githubusercontent.com/ros2/ros2/$ROS_DISTRO-release/ros2.repos \
    && vcs import src < ros2.repos

# install dependencies
RUN apt-get update && rosdep install -y \
    --from-paths src \
    --ignore-src \
    --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 rti-connext-dds-5.3.1 urdfdom_headers" \
    && rm -rf /var/lib/apt/lists/*

# build source
RUN colcon \
    build \
    --symlink-install \
    --mixin build-testing-on release \
    --cmake-args --no-warn-unused-cli

# setup bashrc
RUN cp /etc/skel/.bashrc ~/

# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]