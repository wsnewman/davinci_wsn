cmake_minimum_required(VERSION 2.8.3)
project(dv_string_puller)

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

cs_add_executable(pull_string_hardcoded src/pull_string_hardcoded.cpp)
target_link_libraries(pull_string_hardcoded math_utility davinci_kinematics ${catkin_LIBRARIES})

cs_install()
cs_export()
    
