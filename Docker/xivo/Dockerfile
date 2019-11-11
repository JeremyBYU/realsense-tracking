# This is an auto generated Dockerfile for ros2:devel
# generated from docker_images_ros2/devel/create_ros_image.Dockerfile.em
ARG FROM_IMAGE=jeremybyu/ros2:latest
FROM $FROM_IMAGE
ARG TARGETPLATFORM

RUN sudo apt-get update && \
    apt-get install -q -y libglew-dev

COPY xivo.sh /tmp/  
RUN chmod u+x /tmp/xivo.sh
RUN bash /tmp/xivo.sh ${TARGETPLATFORM} 
