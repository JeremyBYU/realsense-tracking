
# RealSense Robotic Modules

The purpose of this repository is to provide a reproducible (possibly isolated) environment for high level robotics software. Currently the main concern of this repo is interacting with Intel RealSense Cameras.  This environment (code of this repo) can be be distributed as a docker image that is cross-compiled to both x86-64 and arm64 architectures (Regular computers as well as raspberry pi). Here is a quick list of thing this repository is meant to do:

- Have a stable pre-setup ubuntu environment in a docker image with all installed dependencies. 
- Communicate with Intel RealSense Devices using librealsense SDK 2.0.
- Provide a communication framework using [ECAL](https://github.com/continental/ecal) which provides efficient shared memory to publish "messages".
- Provide simple configurations file that can configure publishing of realsense cameras and saving data.
  - See `rspub_default.toml` for publishing and `rssave_default.toml` for saving RealSense data.
- Provide Python client code for visualizing mesh creation.

## Install Libraries and Dependencies

All dependencies and installation procedures can be found in `Docker/base/Dockerfile`. You can use docker and have everything taken care or you can follow the installation procedures in the file. Here is a basic summary of the environment:

- Ubuntu Base Image 18.04
- Python 3 w/ Scipy, numpy
- CMake 3.15.5 - Needed because realsense asks for 3.11 or higher.
- Open CV 3.4.7 with user contributed modules
- ECAL for marshalling and communication
- Protobuf - Serialization format (Thinking of changing to Flatbuffers eventually)
- RealSense SDK
- Open3D - new point cloud and mesh processing library from Intel
- GFLAGS and GLOG for command line parsing and logging

## Setup ECAL

This is only required if you want two computers on the same network to talk to each other.

1. Update each computers hosts file so that they are aware of each other. An example below. Don't forget the `.locadomain`.

```txt
127.0.0.1       localhost
::1             localhost
127.0.1.1       cerberus.localdomain    cerberus
192.168.1.3     raspberrypi

```

2. Configure Multicast for device - `ifconfig YOUR_NETWORK_DEVICE multicast` 
3. Add Multicast route - `sudo route add -net 224.0.0.0 netmask 255.0.0.0 dev YOUR_NETWORK_DEVICE`

Note that I am using the **non-default** multi-cast group `224.0.0.0` for ecal because my router was for some reason blocking the default.

When having any problems first see these issues: [Issue 1](`https://github.com/continental/ecal/issues/38`), [Issue 2](https://github.com/continental/ecal/issues/37)

### ECAL Config File

The software is configured to use the `config/ecal/ecal.ini` file. Note that some values have been changed from the default. If you are using `ecal_mon_gui` on an  external computer to monitor messages dont forget to modify *your* `ecal.ini` to point to the correct UDP multicast address and other options (e.g., `ttl`).

## Run Docker

### X86 Linux

1. [Install Docker](https://docs.docker.com/get-docker/)
2. `git clone --recursive https://github.com/JeremyBYU/realsense-tracking.git && cd realsense-tracking`

### Raspberry PI 4

A raspberry pi image has already been created and set up. Download the image and flash the sd card.

uname: ubuntu

password: pir0b0t

1. `ssh -X ubuntu@192.168.1.25` - Allows XForwarding if needed. Might need to use gui (hdmi) to find out what the ip is first. IP subject to change for your own network.
2. `cd $HOME/Documents/realsense-tracking`

### Launch Docker

1. `rs-pose`, `rs-enumerate-devices` - Need to "open" the sensors on host first. Not sure why.
2. `docker run  --rm --privileged -it --env="DISPLAY" --net=host --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --mount type=bind,source="$(pwd)",target=/opt/workspace --name realsense jeremybyu/realsense:buildx` - Raspberry PI
3. Optional - `rm -rf build && mkdir build && cd build && cmake .. && make && cd ..`

Alternative X86 Launch - `docker run  --rm --privileged -it --env="DISPLAY" --net=host --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --mount type=bind,source="$(pwd)",target=/opt/workspace --name realsense --user $(id -u):$(id -g) jeremybyu/realsense:latest`

## Run Without Docker

Follow `Docker/base/Dockerfile` to install all third party dependencies

1. `mkdir build && cd build`
2. `cmake .. && cmake --build -j8`


## Applications

### RS-Pub

Will publish topics which are configured in a toml file. Configured by `config/rspub_default.toml`.

1. `GLOG_logtostderr=1 ./bin/rs-pub`

`--force_udp` is an option

### RS-Save

Will Subscribe to topics and Save data to disk. Configured by `config/rssave_default.toml`.

1. `GLOG_logtostderr=1 ./bin/rs-save --config=config/rssave_default.toml`

### RS-Proto-Folder

Will convert saved protofiles in a folder to text format. This is for any point clouds or pose information.

1. `python scripts/rs-proto-folder.py`

### RS-Integrate-Server

Creates an RPC Server that allows users to generate meshes on demand with simple requests. This requires both the T265 Poses and D4XX RGBD frames to be published.
It will automatically subscribe to RGBD Images and integrate them to a voxel volume. Users can (on demand) request
to extract meshes, polygons, or even pont clouds from the voxel volume. Configured by `config/rsintegrate_default.toml`.

1. `export OMP_NUM_THREADS=1` - Multithreading actually screws this worse!
2. `GLOG_logtostderr=1 ./bin/rs-integrate-server --v=1`

<!-- ### Reconstruction

This is just an example of doing offline reconstruction in python.

1. `python -m server.ReconstructionSystem.refine_trajectory --config config/reconstruction.json` -->


## Notes

### RPI SSH

#### Copy Saved Mesh File

1. `scp pi@192.168.1.3:/home/pi/Documents/realsense-tracking/data/Default.ply data/Default.ply`


### All dependencies in one folder

This shows a way to run this code here without even needing docker! Just copy the binaries and shared libraries from the docker container and run.
The following code will copy all dependencies into the `dependencies` folder: `./scripts/cpld.sh ./bin/rs-integrate-server ./dependencies`

A couple of caveats: If you are using raspbian and have the 64bit kernel mode enabled then you must use the loader in the dependencies folder to launch the program.  Basically

1. `export LD_LIBRARY_PATH=./dependencies`
1. `GLOG_logtostderr=1 ./dependencies/ld-linux-aarch64.so.1 ./bin/rs-pub`.

Just tested this on ARM and it actually worked! The only thing that was missing was `libecaltime-localtime.so`.

### Trim Down Image

The image is 3.1 GB.  Open3D is about 1.1 GB! 475 MB of that is the python installs from extension (338 MB is just the open3d.so file). Maybe get rid of it?

```txt
Cmp   Size  Command                                                                                  Permission     UID:GID       Size  Filetree
     63 MB  FROM d9bd0dcaa40bc11                                                                     drwx------         0:0      41 MB  ├── root                                                        
    988 kB  [ -z "$(apt-get indextargets)" ]                                                         drwx------         0:0      41 MB  │   └── .cache                                                  
     745 B  set -xe   && echo '#!/bin/sh' > /usr/sbin/policy-rc.d  && echo 'exit 101' >> /usr/sbin/p drwx------         0:0      41 MB  │       └─⊕ pip                                                 
       7 B  mkdir -p /run/systemd && echo 'docker' > /run/systemd/container                          drwxr-xr-x         0:0     1.1 GB  └── usr                                                         
    3.2 MB  echo 'Etc/UTC' > /etc/timezone &&     ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime & drwxr-xr-x         0:0     1.1 GB      └── local                                                   
    446 MB  apt-get update && apt-get install -q -y     bash-completion     lsb-release     python3- drwxr-xr-x         0:0     5.3 kB          ├─⊕ bin                                                 
       0 B  #(nop) WORKDIR /opt/workspace                                                            drwxr-xr-x         0:0      140 B          ├─⊕ etc                                                 
    322 MB  apt-get update &&     apt-get install -q -y wget git libssl-dev libusb-1.0-0-dev pkg-con drwxr-xr-x         0:0      13 MB          ├─⊕ include                                             
    156 MB  apt-get install --no-install-recommends -y build-essential libgtk2.0-dev pkg-config liba drwxr-xr-x         0:0     1.1 GB          ├── lib                                                 
     48 MB  apt-get install --no-install-recommends -y graphviz build-essential zlib1g-dev libhdf5-d drwxr-xr-x         0:0     3.0 kB          │   ├─⊕ cmake                                           
     67 MB  wget -O /opt/cmake-3.15.5.tar.gz https://cmake.org/files/v3.15/cmake-3.15.5.tar.gz &&    -rw-r--r--         0:0     597 MB          │   ├── libOpen3D.a                                     
     745 B  #(nop) COPY file:03028ab537c33176c2c9827484d3abff128258e6c117cf751ff343c7b0df99dc in /tm -rw-r--r--         0:0     4.7 MB          │   ├── libjsoncpp.a                                    
     745 B  chmod u+x /tmp/opencv.sh                                                                 -rw-r--r--         0:0     5.6 MB          │   ├── libqhullcpp.a                                   
    192 MB  git clone -b '3.4.7' --single-branch https://github.com/opencv/opencv.git /opt/opencv && -rw-r--r--         0:0     2.3 MB          │   ├── libqhullstatic_r.a                              
     40 MB  cd /opt &&     git clone --recursive git://github.com/continental/ecal.git &&     cd eca -rw-r--r--         0:0     854 kB          │   ├── libtinyfiledialogs.a                            
    395 MB  git clone https://github.com/IntelRealSense/librealsense.git /opt/librealsense && cd /op -rw-r--r--         0:0     3.1 MB          │   ├── libtinyobjloader.a                              
       0 B  ln -s /usr/bin/python3 /usr/bin/python &     ln -s /usr/bin/pip3 /usr/bin/pip            -rw-r--r--         0:0     765 kB          │   ├── libturbojpeg.a                                  
    1.2 GB  cd /opt && git clone --recursive https://github.com/intel-isl/Open3D &&     cd /opt/Open drwxrwxr-x        0:50     495 MB          │   └── python3.6                                       
    3.4 MB  apt-get install --no-install-recommends -y nano htop                                     drwxrwxr-x        0:50     495 MB          │       └─⊕ dist-packages                               
     32 MB  apt-get install --no-install-recommends -y libgflags-dev libgoogle-glog-dev gfortran     drwxr-xr-x         0:0     4.6 MB          └─⊕ share   

```

