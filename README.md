
# Real Sense Tracking and Depth Modules

The purpose of this repository is to provide an isolated widget that can interact with Intel RealSense Cameras.  The code of this repository is meant to be distributed as a docker image that is cross-compiled to both x86-64 and arm64 architectures (Regular computers as well as raspberry pi). Here is a quick list of thing this repository is meant to do:

- Have a stable pre-setup ubuntu environment in a docker image with all installed dependencies. 
- Communicate with Intel RealSense Devices using librealsense SDK 2.0.
- Provide a marshalling and communication framework using [ECAL](https://github.com/continental/ecal) which provides efficient shared memory to publish RealSense "messages".
- Provide simple configurations file that can configure publishing of realsense cameras and saving data.
  - See `rspub_default.toml` for publishing and `rssave_default.toml` for saving messages.
- Provide Python code for postprocessing the data to demonstrate mesh creation or point cloud alignment.

## Installed libraries and Dependencies

All dependencies and installation procedures can be found in `Docker/base/Dockerfile`. You can use docker and have everything taken care or you can follow the installation procedures. Here is a basic summary of the environment:

- Ubuntu Base Image 18.04
- Python 3 w/ Scipy, numpy
- CMake 3.15.5 - Needed because realsense asks for 3.11 or higher.
- Open CV 3.4.7 with user contributed modules
- ECAL for marshalling and communication
- Protobuf - Serialization format (Thinking of changing to Flatbuffers eventually)
- RealSense SDK
- Open3D - New Point cloud Processing Library from Intel
- GFLAGS and GLOG for command line parsing and logging

## Run Docker

### X86 Linux

1. [Install Docker](https://github.com/continental/ecal)
2. `git clone --recursive https://github.com/JeremyBYU/realsense-tracking.git && cd realsense-tracking`

### Raspberry PI 4

A raspberry pi image has already been created and set up. Download the image and flash the sd card.

uname: ubuntu

password: pir0b0t

1. `ssh -X ubuntu@192.168.1.25` - Allows XForwarding if needed. Might need to use gui (hdmi) to find out what the ip is first. IP subject to change for your own network.
2. `cd $HOME/Documents/realsense-tracking`

### Launch Docker

1. `rs-pose`, `rs-enumerate-devices` - Need to "open" the sensors on host first sometimes?
2. `docker run  --rm --privileged -it --env="DISPLAY" --net=host --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --mount type=bind,source="$(pwd)",target=/opt/workspace --name realsense jeremybyu/realsense:buildx` - Raspberry PI
3. Optional - `rm -rf build && mkdir build && cd build && cmake .. && make && cd ..`

Alternative X86 Launch - `docker run  --rm --privileged -it --env="DISPLAY" --net=host --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --mount type=bind,source="$(pwd)",target=/opt/workspace --name realsense --user $(id -u):$(id -g) jeremybyu/realsense:latest`

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

Creates an RPC Server that allows users to generate meshes on demand with simple requests.
It will automatically subscribe to RGBD Images and integrate them to a voxel volume. Users can (on demand) request
to extract meshes, polygons, or even pont clouds from the voxel volume. Configured by `config/rsintegrate_default.toml`.

1. `export OMP_NUM_THREADS=1` - Multithreading actually screws this worse!
2. `GLOG_logtostderr=1 ./bin/rs-integrate-server --v=1`

### Reconstruction

This is just an example of doing offline reconstruction in python.

1. `python -m server.ReconstructionSystem.refine_trajectory --config config/reconstruction.json`

### RS-Save-Python (Beta)

Doesn't work in docker. Dont recommend to use this, use RS-Save.

1. `cd server/ReconstrucitonSystem`
2. `python sensors/realsense_recorder` some options

## Notes

### RPI SSH

#### Copy Saved Mesh File

1. `scp pi@192.168.1.3:/home/pi/Documents/realsense-tracking/data/Default.ply data/Default.ply`

### RealSense

Need to use development branch of realsense:

```
commit 306e68aca67bd0ecaf5bb40ad6266f4ce8e98690 (HEAD -> development, origin/development)
Merge: 6d6ead55a e5692637f
Author: Sergey Dorodnicov <sergey.dorodnicov@intel.com>
Date:   Tue Jan 7 17:17:02 2020 +0200

    Merge pull request #5504 from radfordi/t265-release-908
    
     Upgrade T265 firmware to 0.2.0.908
```

D435i SN - 844212071822
T265 SN - 943222110884, 0000943222110884

### All dependencies in one folder

This shows a way to run this code here without even needing docker! Just copy the binaries and shared libraries from the docker container and run.

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

### Integration

What I have learned. If you are going to integrate point clouds over time you **need** to use a proper integration method like TSDF Volume Integration. It will smooth out gaussian noise and provide a much better estimate of the environment. This process creates an integrated (read memory efficient) voxel map of the environment and can be "quickly" transformed into a pont cloud or mesh.

If you just try to use slam and cast your point clouds from your noisy sensor it will work, but the noise is compounded! Floor have "layers" to them. Polylidar will just choke on noisy dense point clouds! Polylidar can handle sparse noisy point clouds and dense noisy point cloud at one *instant* in time (just downsample to make less dense).
