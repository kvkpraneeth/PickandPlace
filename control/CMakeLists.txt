cmake_minimum_required(VERSION 2.8.3)

project(control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    gazebo_ros 
    roscpp
    moveit_core 
    moveit_ros_planning 
    moveit_ros_planning_interface 
)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

add_library(control_core src/control.cpp)
target_link_libraries(control_core ${catkin_LIBRARIES})

add_executable(control src/main.cpp)
target_link_libraries(control control_core ${catkin_LIBRARIES})
add_dependencies(control control_core)

