cmake_minimum_required(VERSION 2.8.3)
project(pcl_ransac)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINTIONS})

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

cs_add_library(${PROJECT_NAME}_lib src/pcl_ransac.cpp)
target_link_libraries(${PROJECT_NAME}_lib ${PCL_LIBRARIES} ${Eigen3_LIBRARIES})

cs_install()
cs_export()

if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(${PROJECT_NAME}-test test/src/pcl_ransac_test.cpp)
    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_lib)
endif()