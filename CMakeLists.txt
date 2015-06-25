cmake_minimum_required (VERSION 2.6)
project(ec_car_detection)

# Project directory
add_subdirectory(src)

# Dependencies
add_subdirectory(lib/bovil/src)
add_subdirectory(lib/DroneApplications/src)
add_subdirectory(lib/ardronepp/src)
