cmake_minimum_required(VERSION 2.8.3)
project(mpu9250_node)
set (CMAKE_CXX_STANDARD 11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_srvs roslint)

## Declare a catkin package
catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS roscpp sensor_msgs std_srvs)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(mpu9250_node src/mpu9250_node.cpp)
target_link_libraries(mpu9250_node ${catkin_LIBRARIES} -lupm-mpu9150)

roslint_cpp()
