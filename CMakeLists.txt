cmake_minimum_required(VERSION 3.1.0)
project(realsensetracking)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++17 -Wno-narrowing -Wno-register -fPIC -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mtune=native -march=native")
set(CMAKE_BUILD_TYPE "Release")

set(PROTO_DIR "${PROJECT_SOURCE_DIR}/proto")

add_definitions(-DNDEBUG)

set(toml11_BUILD_TEST OFF CACHE BOOL "Build toml tests" FORCE)
add_subdirectory("thirdparty/toml11")

find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(Protobuf REQUIRED)
find_package(eCAL REQUIRED)
find_package(OpenGL)
find_package(gflags REQUIRED)
find_package(glog) # Docker ubuntu can not find when REQUIRED, but logging still works?!?!?
# find_package( Eigen3 REQUIRED )
find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/lib/CMake)

set(PROTO_FILES ${PROTO_DIR}/Common.proto  ${PROTO_DIR}/ImageMessage.proto 
                ${PROTO_DIR}/PointCloudMessage.proto  ${PROTO_DIR}/IMUMessage.proto 
                ${PROTO_DIR}/PoseMessage.proto ${PROTO_DIR}/Integrate.proto)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})
protobuf_generate_python(PY_SOURCES ${PROTO_FILES})
# Print path to generated files
message ("PROTO_SRCS = ${PROTO_SRCS}")
message ("PROTO_HDRS = ${PROTO_HDRS}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

set(DEPENDENCIES glfw realsense2-gl ${OPENGL_LIBRARIES} ${DEPENDENCIES} ${PROTOBUF_LIBRARIES} ${Open3D_LIBRARIES} ${OpenCV_LIBRARIES} gflags glog eCAL::core) 


get_cmake_property(_variableNames VARIABLES)
list (SORT _variableNames)
foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
endforeach()

link_directories(
    /usr/lib
    /usr/lib/x86_64-linux-gnu
    /usr/local/lib
    ${DEPENDENCIES}
    ${Open3D_LIBRARY_DIRS}
    # ${XIVO_DIR}/thirdparty/gflags/lib
    # ${XIVO_DIR}/thirdparty/glog/lib
    # ${XIVO_DIR}/thirdparty/Pangolin/lib
    # ${XIVO_DIR}/thirdparty/jsoncpp/lib
    # ${XIVO_DIR}/lib
)

include_directories(
    ${PROJECT_SOURCE_DIR}/src/include
    /usr/include
    /usr/local/include
    /usr/local/include/opencv2
    ${PROJECT_SOURCE_DIR}/../librealsense/common/res
    ${PROJECT_SOURCE_DIR}/../librealsense/src
    # ${EIGEN3_INCLUDE_DIR}

    ${PROTOBUF_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/build
    ${GFLAG_INCLUDE_DIRS}
    ${toml11_SOURCE_DIR}
    ${Open3D_INCLUDE_DIRS}
)


# Find librealsense2 installed package

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Add the application sources to the target
add_executable(rs-motion-2 src/rs-motion-2.cpp)
add_executable(rs-test src/rs-test.cpp src/utility.cpp ${PROTO_SRCS} ${PROTO_HDRS})
add_executable(rs-pub src/rs-pub.cpp src/utility.cpp ${PROTO_SRCS} ${PROTO_HDRS})
add_executable(rs-save src/rs-save.cpp src/utility.cpp ${PROTO_SRCS} ${PROTO_HDRS})
add_executable(rs-integrate-server src/rs-integrate-server.cpp src/utility.cpp ${PROTO_SRCS} ${PROTO_HDRS})

# Link librealsense2 to the target
target_link_libraries(rs-motion-2 ${realsense2_LIBRARY} ${DEPENDENCIES})
target_link_libraries(rs-test ${realsense2_LIBRARY} ${DEPENDENCIES}) 
target_link_libraries(rs-pub ${realsense2_LIBRARY} ${DEPENDENCIES})
target_link_libraries(rs-save ${realsense2_LIBRARY} ${DEPENDENCIES})
target_link_libraries(rs-integrate-server ${realsense2_LIBRARY} ${DEPENDENCIES} )

add_custom_target(DUMMY_TARGET ALL 
                  DEPENDS ${PY_SOURCES})