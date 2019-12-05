cmake_minimum_required(VERSION 3.1.0)
project(realsensetracking)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++14 -Wno-narrowing -Wno-register -fPIC -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mtune=native -march=native")
set(CMAKE_BUILD_TYPE "Release")
set(XIVO_DIR "/opt/xivo")
set(PROTO_DIR "${PROJECT_SOURCE_DIR}/src/proto")

add_definitions(-DNDEBUG)

find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(OpenGL)
find_package(Protobuf REQUIRED)
add_subdirectory(${XIVO_DIR} build)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_DIR}/ImageData.proto)
# Print path to generated files
message ("PROTO_SRCS = ${PROTO_SRCS}")
message ("PROTO_HDRS = ${PROTO_HDRS}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

set(DEPENDENCIES glfw realsense2-gl ${OPENGL_LIBRARIES} ${DEPENDENCIES} ${libxivo} ${PROTOBUF_LIBRARIES} gflags::gflags) 

link_directories(
    /usr/lib
    /usr/lib/x86_64-linux-gnu
    /usr/local/lib
    ${DEPENDENCIES}
    ${XIVO_DIR}/thirdparty/gflags/lib
    ${XIVO_DIR}/lib
)

include_directories(
    ${PROJECT_SOURCE_DIR}/src/include
    /usr/include
    /usr/local/include
    /usr/local/include/opencv2
    ${PROJECT_SOURCE_DIR}/../librealsense/common/res
    ${PROJECT_SOURCE_DIR}/../librealsense/src
    ${XIVO_DIR}/thirdparty/gflags/include
    ${XIVO_DIR}/common
    ${XIVO_DIR}/src
    ${PROTOBUF_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/build
)


# Find librealsense2 installed package

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Add the application sources to the target
add_executable(rs-motion src/rs-motion.cpp)
add_executable(rs-motion-2 src/rs-motion-2.cpp)
add_executable(rs-tracker src/rs-tracker.cpp src/utility.cpp ${PROTO_SRCS} ${PROTO_HDRS})

# Link librealsense2 to the target
target_link_libraries(rs-motion ${realsense2_LIBRARY} ${DEPENDENCIES})
target_link_libraries(rs-motion-2 ${realsense2_LIBRARY} ${DEPENDENCIES})
target_link_libraries(rs-tracker ${realsense2_LIBRARY} ${DEPENDENCIES})