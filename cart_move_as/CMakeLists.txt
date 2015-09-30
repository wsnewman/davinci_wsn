cmake_minimum_required(VERSION 2.8.3)
project(cart_move_as)

find_package(catkin_simple REQUIRED)
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
# cs_add_library(my_lib src/my_lib.cpp)   

# Executables
cs_add_executable(cart_move_as src/cart_move_as.cpp)
cs_add_executable(cart_move_client_example src/cart_move_client_example.cpp)
target_link_libraries(cart_move_as davinci_kinematics ${catkin_LIBRARIES})
target_link_libraries(cart_move_client_example ${catkin_LIBRARIES})
cs_install()
cs_export()
    
