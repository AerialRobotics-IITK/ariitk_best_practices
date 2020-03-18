cmake_minimum_required(VERSION 2.8.3)
project(lib_package_template)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINTIONS})

cs_add_library(${PROJECT_NAME}_lib src/template.cpp)
target_link_libraries(${PROJECT_NAME}_lib ${PCL_LIBRARIES})

cs_install()
cs_export()

if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(${PROJECT_NAME}-test test/src/template_test.cpp)
    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_lib)
endif()