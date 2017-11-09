cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(VehiclePluginMod SHARED VehiclePluginMod.cc)
target_link_libraries(VehiclePluginMod ${GAZEBO_libraries})

# Build the stand-alone test program
add_executable(vel_VehiclePluginMod vel_VehiclePluginMod.cc)
target_link_libraries(vel_VehiclePluginMod ${GAZEBO_LIBRARIES})

