FROM jeremybyu/realsense:buildx
ARG N_CPUS
# Clone Realsense - Depth, IMU, 6DOF Sensor SDK
RUN git clone --single-branch --branch v2.40.0 --recursive https://github.com/IntelRealSense/librealsense.git /opt/librealsense && \
    cd /opt/librealsense && mkdir build && cd build && \
    cmake ../ -DFORCE_RSUSB_BACKEND=true -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true -DBUILD_PYTHON_BINDINGS=true && \
    make -j${N_CPUS}&& \
    make install && \
    echo "from .pyrealsense2 import *" >> /usr/lib/python3/dist-packages/pyrealsense2/__init__.py && \
    rm -rf /opt/librealsense

ENV PYTHONPATH $PYTHONPATH:/usr/local/lib

