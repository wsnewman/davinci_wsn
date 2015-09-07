cmake_minimum_required(VERSION 2.8.3)
project(davinci_kinematics)

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})


catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
cs_add_library(davinci_kinematics src/davinci_kinematics.cpp)   
cs_add_library(davinci_joint_publisher src/davinci_joint_publisher.cpp) 
# Executables
cs_add_executable(test_davinci_ik src/davinci_kinematics_test_main.cpp)
cs_add_executable(test_davinci_joint_pub src/davinci_joint_publisher_test_main.cpp)
#cs_add_executable(davinci_cart_traj_pub src/davinci_cart_traj_pub.cpp)
cs_add_executable(davinci_playfile_jointspace src/davinci_playfile_jointspace.cpp)
target_link_libraries(test_davinci_ik davinci_kinematics ${catkin_LIBRARIES})
target_link_libraries(test_davinci_joint_pub davinci_joint_publisher ${catkin_LIBRARIES})
#target_link_libraries(davinci_cart_traj_pub davinci_joint_publisher davinci_kinematics ${catkin_LIBRARIES})
target_link_libraries(davinci_playfile_jointspace davinci_joint_publisher davinci_kinematics ${catkin_LIBRARIES})

# target_link_library(example my_lib)

cs_install()
cs_export()
    
