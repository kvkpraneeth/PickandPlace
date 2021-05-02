cmake_minimum_required(VERSION 2.8.3)

project(simenv)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS urdf gazebo_ros)

catkin_package()
