cmake_minimum_required(VERSION 3.14)

# Project Details
set(PROJECT_NAME "RoboEnv")
set(PROJECT_VERSION 1.0.0)

# Set Project Properties
project(${PROJECT_NAME} VERSION ${PROJECT_VERSION}
DESCRIPTION "Robotic Environment: RealSense, ECAL, OpenCV, etc." LANGUAGES CXX)

# Set Global Properties of Compiler
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# add_definitions(-DNDEBUG)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++17 -Wno-narrowing -Wno-register -fPIC -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mtune=native -march=native")
set(CMAKE_BUILD_TYPE "Release")


# Add any "git cloned" dependencies needed by our project. They will be declared in "thridparty" directory
add_subdirectory("thirdparty")

# Add any dependencies that have been installed into standard default locations (e.g., /usr/local/lib)
# These dependencies should have been installed by Docker (`Docker/base/Dockerfile`)
# Or by the user by following the Dockerfile
find_package(OpenCV REQUIRED)
find_package(OpenMP)
find_package(realsense2 REQUIRED)
find_package(Protobuf REQUIRED)
find_package(eCAL COMPONENTS core REQUIRED)
find_package(OpenGL)
find_package(gflags REQUIRED)
find_package(glog) # Docker ubuntu can not find when REQUIRED, but logging still works?!?!?
find_package(Eigen3 REQUIRED )
find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/lib/CMake REQUIRED)



# Protobuff Generation
set(PROTO_DIR "${PROJECT_SOURCE_DIR}/proto")
set(PROTO_FILES ${PROTO_DIR}/Common.proto  ${PROTO_DIR}/ImageMessage.proto 
                ${PROTO_DIR}/PointCloudMessage.proto  ${PROTO_DIR}/IMUMessage.proto 
                ${PROTO_DIR}/PoseMessage.proto ${PROTO_DIR}/Integrate.proto)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})
protobuf_generate_python(PY_SOURCES ${PROTO_FILES})
# Print path to generated files
message ("PROTO_SRCS = ${PROTO_SRCS}")
message ("PROTO_HDRS = ${PROTO_HDRS}")

# Set the output directories of cmake
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)





# get_cmake_property(_variableNames VARIABLES)
# list (SORT _variableNames)
# foreach (_variableName ${_variableNames})
#     message(STATUS "${_variableName}=${${_variableName}}")
# endforeach()

# We should really be using target_link_directories
# but since this is an *application* repository and there will be 
# many binaries, I dont see any harm.
link_directories(
    /usr/lib
    /usr/lib/x86_64-linux-gnu
    /usr/local/lib
    ${DEPENDENCIES}
    ${Open3D_LIBRARY_DIRS}
)

include_directories(
    ${PROJECT_SOURCE_DIR}/src/include
    /usr/include
    /usr/local/include
    /usr/local/include/opencv2
    ${PROJECT_SOURCE_DIR}/../librealsense/common/res
    ${PROJECT_SOURCE_DIR}/../librealsense/src
    ${EIGEN3_INCLUDE_DIR}

    ${PROTOBUF_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/build
    ${GFLAG_INCLUDE_DIRS}
    ${toml11_SOURCE_DIR}
    ${Open3D_INCLUDE_DIRS}
)


# Set all the dependencies
set(DEPENDENCIES glfw realsense2-gl ${OPENGL_LIBRARIES} ${DEPENDENCIES} ${PROTOBUF_LIBRARIES} 
                 ${Open3D_LIBRARIES} ${OpenCV_LIBRARIES} OpenMP::OpenMP_CXX gflags glog eCAL::core) 

# Add the application sources to the target
# add_executable(rs-motion-2 src/rs-motion-2.cpp)
# add_executable(rs-test src/rs-test.cpp src/utility.cpp ${PROTO_SRCS} ${PROTO_HDRS})
add_executable(rs-pub src/rs-pub.cpp src/utility.cpp ${PROTO_SRCS} ${PROTO_HDRS})
add_executable(rs-save src/rs-save.cpp src/utility.cpp ${PROTO_SRCS} ${PROTO_HDRS})
add_executable(rs-integrate-server src/rs-integrate-server.cpp src/utility.cpp ${PROTO_SRCS} ${PROTO_HDRS})

# Link the Libraries
# target_link_libraries(rs-motion-2 ${realsense2_LIBRARY} ${DEPENDENCIES})
# target_link_libraries(rs-test ${realsense2_LIBRARY} ${DEPENDENCIES}) 
target_link_libraries(rs-pub ${realsense2_LIBRARY} ${DEPENDENCIES})
target_link_libraries(rs-save ${realsense2_LIBRARY} ${DEPENDENCIES})
target_link_libraries(rs-integrate-server ${realsense2_LIBRARY} ${DEPENDENCIES})

# Must have this dummy target for the python generated sources by Probobuff CMake.
add_custom_target(DUMMY_TARGET ALL 
                  DEPENDS ${PY_SOURCES})