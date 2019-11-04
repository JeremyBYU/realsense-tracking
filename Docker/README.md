
## How to build

Taken from here mostly: https://engineering.docker.com/2019/06/getting-started-with-docker-for-arm-on-linux/

1. Install docker
2. `export DOCKER_CLI_EXPERIMENTAL=enabled` - Enable experimental buildx commands
3. `docker run --rm --privileged docker/binfmt:820fdd95a9972a5308930a2bdfb8573dd4447ad3`
4. `docker buildx create --name mybuilder`
5. `docker buildx use mybuilder`
6. `docker buildx inspect --bootstrap`

Actually build dockerimage 

1. `cd Docker` - Make sure your in the Docker directory
2. `chmod +x ros_entrypoint.sh` - Make sure this is executable
3. `docker buildx build --platform linux/arm64 -t jeremybyu/ros2 --output type=docker . `

Running the image as a container
1. cd `realsense-tracking` - The main direcotry
1. `docker run --rm -it -v "$(pwd)/src:/opt/ros2_ws/src" jeremybyu/ros2`