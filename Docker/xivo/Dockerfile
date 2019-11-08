# This is an auto generated Dockerfile for ros2:devel
# generated from docker_images_ros2/devel/create_ros_image.Dockerfile.em
ARG FROM_IMAGE=jeremybyu/test:latest
FROM $FROM_IMAGE
ARG TARGETPLATFORM

COPY xivo.sh /tmp/  
RUN chmod u+x /tmp/xivo.sh
RUN bash /tmp/xivo.sh ${TARGETPLATFORM} 


ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]