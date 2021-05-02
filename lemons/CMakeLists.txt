cmake_minimum_required(VERSION 2.8.3)

project(lemons)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS lemons_moveit simenv)

catkin_package()

