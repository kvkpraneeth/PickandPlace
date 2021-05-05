cmake_minimum_required(VERSION 2.8.3)

project(perception)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge geometry_msgs message_generation)

add_service_files(FILES getTransform.srv)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_library(vision_core src/vision.cpp)
target_link_libraries(vision_core ${catkin_LIBRARIES})

add_executable(vision src/main.cpp)
target_link_libraries(vision vision_core ${catkin_LIBRARIES})
add_dependencies(vision vision_core)
