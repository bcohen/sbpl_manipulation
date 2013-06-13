cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

#Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

# Add a ROS C++ macro to tell SBPL to use ROS print statements
add_definitions(-DROS)

rosbuild_init()

rosbuild_add_boost_directories()

#find_package(Eigen REQUIRED)
#include_directories(${EIGEN_INCLUDE_DIRS})
#add_definitions(${EIGEN_DEFINITIONS})

#rosbuild_add_compile_flags(-g)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(sbpl_manipulation_components 
        src/robot_model.cpp
        src/kdl_robot_model.cpp
        src/occupancy_grid.cpp
        src/collision_checker.cpp
        src/post_processing.cpp)

target_link_libraries(sbpl_manipulation_components sbpl_geometry_utils)

rosbuild_add_executable(test_kdl src/test_kdl_robot_model.cpp)
target_link_libraries(test_kdl sbpl_manipulation_components)
