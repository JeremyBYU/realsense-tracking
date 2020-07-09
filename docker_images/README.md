# Instructions

You can save a docker image to a file as so:
docker save -o <path for generated tar file> <image name>
docker save -o docker_images/roboenv.tar jeremybyu/realsense:latest

You can load a docker image as so:
docker load -i roboenv.tar