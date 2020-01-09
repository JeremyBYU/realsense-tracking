

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

1. `docker run  --rm --privileged -it --env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="$(pwd):/opt/workspace:rw" jeremybyu/realsense:buildx`
2. Optional - `rm -rf build && mkdir build && cd build && cmake .. && make && cd ..`

# Notes

D435i SN - 844212071822
T265 SN - 943222110884, 0000943222110884
