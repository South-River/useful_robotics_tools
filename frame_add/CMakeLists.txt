cmake_minimum_required(VERSION 3.0.2)
project(frame_add)

set(CMAKE_VERBOSE_MAKEFILE "false")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17 -march=native ${CMAKE_CXX_FLAGS} -g -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf2
    tf2_ros
)

# find_package(tf2 REQUIRED)
# find_package(tf2_ros REQUIRED)

find_package(Eigen3 REQUIRED)

catkin_package(
    # INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp
)

include_directories(
    SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}
    src/frame_add.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)