
## How to prepare crossplatform build

Taken from here mostly: https://engineering.docker.com/2019/06/getting-started-with-docker-for-arm-on-linux/

Also read this: https://medium.com/@artur.klauser/building-multi-architecture-docker-images-with-buildx-27d80f7e2408

1. Install docker
2. `export DOCKER_CLI_EXPERIMENTAL=enabled` - Enable experimental buildx commands
3. `docker run --rm --privileged docker/binfmt:820fdd95a9972a5308930a2bdfb8573dd4447ad3`
4. `docker buildx create --name mybuilder`
5. `docker buildx use mybuilder`
6. `docker buildx inspect --bootstrap`

### Actually build dockerimage 

1. `cd Docker/base` - Make sure your in the Docker directory
2. `docker buildx build --platform linux/arm64 -t jeremybyu/realsense:buildx --output type=docker . ` - Build local arm instance
3. `docker buildx build --platform linux/arm64 -t jeremybyu/realsense:buildx --push .`

### Running the arm64 docker container on x86 linux host

1. cd `realsense-tracking` - The main directory
2. `docker run  --rm --privileged -it --env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="$(pwd):/opt/workspace:rw" jeremybyu/realsense:buildx`

## Build with regular x86 docker

1. `cd Docker/base`
2. `docker build -t jeremybyu/realsense:latest .`

### Running the image as a container

1. cd `realsense-tracking` - The main directory
2. `docker run --rm --privileged -it --env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="$(pwd):/opt/workspace:rw" jeremybyu/realsense:latest`
