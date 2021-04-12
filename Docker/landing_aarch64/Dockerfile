ARG N_CPUS=12
FROM jeremybyu/realsense:buildx
ARG N_CPUS

# This is needed from before, just in case environment variabled didn't carry forth from jeremybyu/realsense
ENV PYTHONPATH $PYTHONPATH:/usr/local/lib

# install packages
RUN apt-get update && apt-get install -q -y \
    python3-shapely \
    python3-matplotlib \
    gfortran \
    libopenblas-dev \
    liblapack-dev \
    cython3

RUN apt-get -q -y purge python3-scipy

RUN pip install toml pyyaml
RUN pip install scipy==1.4.1 --no-binary :all: --no-cache-dir --compile

# Clone Polylidar3D
RUN git clone --recursive https://github.com/JeremyBYU/polylidar.git /opt/polylidar && \
    cd /opt/polylidar && mkdir cmake-build && cd cmake-build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DFETCHCONTENT_QUIET=OFF && \
    make -j${N_CPUS} && \
    cmake --build . --target python-package --config Release -j${N_CPUS} && \
    cd lib/python_package && pip install . && \
    rm -rf /opt/polylidar


# Clone FastGaussianAccumulator
RUN git clone -b 's2beta' --single-branch --recursive https://github.com/JeremyBYU/FastGaussianAccumulator.git /opt/fastga && \
    cd /opt/fastga && mkdir cmake-build && cd cmake-build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DFETCHCONTENT_QUIET=OFF -DGA_BUILD_TESTS=OFF -DGA_WITH_OPENMP=OFF -DGA_BUILD_BENCHMARKS=OFF -DGA_BUILD_EXAMPLES=OFF  && \
    make -j${N_CPUS} && \
    cmake --build . --target python-package --config Release -j${N_CPUS} && \
    cd lib/python_package && pip install . && \
    rm -rf /opt/fastga

# Clone OrganizedPointFilters
RUN git clone -b 'rpi4' --single-branch --recursive https://github.com/JeremyBYU/OrganizedPointFilters.git /opt/opf && \
    cd /opt/opf && mkdir cmake-build && cd cmake-build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DFETCHCONTENT_QUIET=OFF -DOPF_BUILD_TESTS=OFF -DOPF_WITH_OPENMP=OFF -DOPF_BUILD_BENCHMARKS=OFF -DOPF_BUILD_EXAMPLES=OFF && \
    make -j${N_CPUS} && \
    cmake --build . --target python-package --config Release -j${N_CPUS} && \
    cd lib/python_package && pip install . && \
    rm -rf /opt/opf

RUN pip install multiprocessing-logging pyserial

# Polylabel Fast
RUN git clone --recursive https://github.com/JeremyBYU/polylabelfast.git /opt/polylabelfast && \
    cd /opt/polylabelfast && pip install . && \
    rm -rf /opt/polylabelfast
