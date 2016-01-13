cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(uarm_plugin)

# Find packages

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})


find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()


# include appropriate directories
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

set(UNFILTERED_FLAGS "-std=gnu++11")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}${WARNING_CXX_FLAGS} ${UNFILTERED_FLAGS}")

# Create libraries and executables

add_library(UarmPlugin SHARED UarmPlugin.cc)
target_link_libraries(UarmPlugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_executable(set_pose set_pose.cc)
target_link_libraries(set_pose ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} pthread)

