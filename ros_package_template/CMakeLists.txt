cmake_minimum_required(VERSION 2.8.3)
project(ros_package_template)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINTIONS})

cs_add_library(${PROJECT_NAME}_lib src/template_ros.cpp src/template_ros_pub.cpp)
target_link_libraries(${PROJECT_NAME}_lib ${PCL_LIBRARIES})

cs_add_executable(${PROJECT_NAME}_node src/template_ros_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib) 

cs_install()
cs_export()