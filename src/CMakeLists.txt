cmake_minimum_required (VERSION 2.6)
project(car_detection)

# Collect all sources
file(GLOB_RECURSE CARDETECTION_SOURCE_FILES "code/*.cpp" "code/*.h" "code/*.inl")

# Clasify sources according to folder structure.
# This creates a nice layout for the project in visual studio
# This macro is courtesy of http://www.cmake.org/pipermail/cmake/2013-November/056336.html
macro(GroupSources curdir)
	file(GLOB children RELATIVE ${PROJECT_SOURCE_DIR}/${curdir}
		${PROJECT_SOURCE_DIR}/${curdir}/*)
	foreach(child ${children})
		if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/${curdir}/${child})
			GroupSources(${curdir}/${child})
		else()
			string(REPLACE "/" "\\" groupname ${curdir})
			source_group(${groupname} FILES ${PROJECT_SOURCE_DIR}/${curdir}/${child})
		endif()
	endforeach()
endmacro()

macro(IncludeFromEnvironment envVar)
	set(INCLUDE_DIR ${envVar})
	if(DEFINED INCLUDE_DIR)
		include_directories(${INCLUDE_DIR})
	else()
		message(SEND_ERROR "Cant locate eigen include directory. Please define an environment variable with include directory")
	endif()
endmacro()

GroupSources(code)

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
add_executable(car_detection ${CARDETECTION_SOURCE_FILES})

target_include_directories(car_detection PUBLIC ${PROJECT_SOURCE_DIR}/code)
target_include_directories(car_detection PUBLIC ${PROJECT_SOURCE_DIR}/../../lib/bovil/src/code)
target_include_directories(car_detection PUBLIC ${PROJECT_SOURCE_DIR}/../../lib/DroneApplications/src/code)
target_link_libraries(car_detection LINK_PUBLIC BOViL)
target_link_libraries(car_detection LINK_PUBLIC DroneApplications)
target_link_libraries(car_detection LINK_PUBLIC ${OPENCV_LIBS})


file(COPY ../data.json DESTINATION ${CMAKE_BINARY_DIR}/src)