cmake_minimum_required(VERSION 2.8.3)
project(other_utilities)

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

cs_add_library(math_utility src/math_utility.cpp)

cs_install()
cs_export()
