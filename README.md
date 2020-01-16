
# Real Sense Tracking and Depth Modules

The purpose of this repository is to provide an isolated widget that can interact with Intel RealSense Cameras.  The code of this repository is meant to be distributed as a docker image that is cross-compiled to both x86-64 and arm64 architectures (Regular computers as well as raspberry pi). Here is a quick list of thing this repository is meant to do:


- Have a stable pre-setup ubuntu environment in a docker image with all installed depenedencies. 
- Communicate with Intel RealSense Devices using librealsense SDK 2.0.
- Provide a marshalling and communication framework using [ECAL](https://github.com/continental/ecal) which povides effecient shared memory to publish RealSense "messages".
- Provide simple configurations file that can configure publishing of realsense cameras and saving data.
  - See `rspub_default.toml` for publishing and `rssave_default.toml` for saving messages.
- Provide Python code for postprocessing the data to demonstrate mesh creation or point cloud alignment.

## Installed libraries and Dependencies

All dependencies and installation procedures can be found in `Docker/base/Dockerfile`. You can use docker and have everything taken care or you can follow the installation procedures. To provide a basic summary of the environment:

- Ubuntu Base Image 18.04
- Python 3 w/ Scipy, numpy
- CMake 3.15.5 - Needed because realsense asks for 3.11 or higher.
- Open CV 3.4.7 with user contributed modules
- ECAL for marshalling and communication
- Protobuf - Thinking of changing to Flatbuffers eventually
- RealSense SDK
- Open3D - Point cloud Processing Library
- GFLAGS and GLOG for command line parsing and logging


## Run Docker

### X86 Linux

1. [Install Docker](https://github.com/continental/ecal)
2. `git clone https://github.com/JeremyBYU/realsense-tracking.git && cd realsense-tracking`

### Rasperry PI 4

A rasberry pi image has already been created and set up. Download the image and flash the sd card.

uname: ubuntu

password: pir0b0t

**SSH**
1. `ssh -X ubuntu@192.168.1.25` - Allows XForwarding. Might need to use gui (hdmi) to find out what the ip is first. IP subject to change for your own network.
2. `cd $HOME/Documents/realsense-tracking`

### Launch Docker

1. `rs-pose`, `rs-enumerate-devices` - Need to "open" the sensors on host first?
2. `docker run  --rm --privileged -it --env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="$(pwd):/opt/workspace:rw" jeremybyu/realsense:buildx` - Instruction for Raspberry pi
3. Optional - `rm -rf build && mkdir build && cd build && cmake .. && make && cd ..`

Alternative X86 Launch - `docker run  --rm --privileged -it --env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --mount type=bind,source="$(pwd)",target=/opt/workspace --name realsense --user $(id -u):$(id -g) jeremybyu/realsense:latest`

## Applications

### RS-Pub

Will publish topics which are configured in a toml file

1. `GLOG_logtostderr=1 ../bin/rs-pub`

### RS-Save

Will Subscribe to topics and Save

1. `GLOG_logtostderr=1 ./bin/rs-save --config=config/rssave_default.toml`

### RS-Proto-Folder

Will convert saved protofiles in a folder to text format

1. `python scripts/rs-proto-folder.py`

### Reconstruction

Doesnt work in docker. Need Open3D and Scipy

1. `python -m server.ReconstructionSystem.refine_trajectory --config config/reconstruction.json`

### RS-Save-Pyton (Beta)

Doesnt work in docker

1. `cd server/ReconstrucitonSystem`
2. `python sensors/realsense_recorder` some options

## Notes

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

1. `./scripts/cpld.sh ./bin/rs-track-depth ./dependencies`
2. `LD_LIBRARY_PATH=./dependencies:$LD_LIBRARY_PATH GLOG_logtostderr=1 ./bin/rs-track-depth`

No idea if this should work, need to try out on an untainted ubuntu 18.04 5.3 kernel computer.
