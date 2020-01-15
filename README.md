

# Run Docker

## X86

1. Install Docker
2. `git clone https://github.com/JeremyBYU/realsense-tracking.git && cd realsense-tracking`

## Rasperry PI 4

A rasberry pi image has already been created and set up. Download the image and flash the sd card.

uname: ubuntu
password: pir0b0t

**SSH**
1. `ssh -X ubuntu@192.168.1.25` - Allows XForwarding. Might need to use gui (hdmi) to find out what the ip is first.
2. `cd $HOME/Documents/realsense-tracking`

## Launch Docker

1. `rs-pose`, `rs-enumerate-devices` - Need to "open" the sensors on host first?
2. `docker run  --rm --privileged -it --env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="$(pwd):/opt/workspace:rw" jeremybyu/realsense:buildx`
3. Optional - `rm -rf build && mkdir build && cd build && cmake .. && make && cd ..`

# Applications

## RS-Pub

Will publish topics which are configured in a toml file

1. `GLOG_logtostderr=1 ../bin/rs-pub`

## RS-Save

Will Subscribe to topics and Save

1. `GLOG_logtostderr=1 ./bin/rs-save --config=config/rssave_default.toml`

## RS-Proto-Folder

Will convert saved protofiles in a folder to text format

1. `python scripts/rs-proto-folder.py`

## Reconstruction

1. `python -m server.ReconstructionSystem.refine_trajectory --config config/reconstruction.json`

# Notes

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

## All dependencies in one folder

1. `./scripts/cpld.sh ./bin/rs-track-depth ./dependencies`
2. `LD_LIBRARY_PATH=./dependencies:$LD_LIBRARY_PATH GLOG_logtostderr=1 ./bin/rs-track-depth`

No idea if this should work, need to try out on an untainted ubuntu 18.04 5.3 kernel computer.
