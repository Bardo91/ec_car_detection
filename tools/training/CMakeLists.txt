cmake_minimum_required (VERSION 2.6)
project(training)

# Collect all sources
file(GLOB_RECURSE CARDETECTION_SOURCE_FILES "main.cpp")

# Compiler specific options
if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=c++11)
endif()

# Getting module from environment.
set(CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH})

# Loading 3rd party libraries.
find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIR})
include_directories(${OPENCV_LIB_DIR})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Add Executable
add_executable(training ${CARDETECTION_SOURCE_FILES})

target_include_directories(training PUBLIC ${PROJECT_SOURCE_DIR}/)
target_include_directories(training PUBLIC ${PROJECT_SOURCE_DIR}/../../lib/bovil/src/code)
target_include_directories(training PUBLIC ${PROJECT_SOURCE_DIR}/../../lib/DroneApplications/src/code)
target_link_libraries(training LINK_PUBLIC BOViL)
target_link_libraries(training LINK_PUBLIC DroneApplications)
target_link_libraries(training LINK_PUBLIC ${OPENCV_LIBS})