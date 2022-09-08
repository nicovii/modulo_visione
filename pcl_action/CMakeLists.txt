cmake_minimum_required(VERSION 3.0.2)
project(pcl_action)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  roscpp
  std_msgs
  darknet_ros_msgs
  message_generation
)

add_message_files(
  FILES
  custom.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

find_package(
  Eigen3 REQUIRED
)

find_package(PCL 1.2 REQUIRED)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl_action
  src/main.cpp 
)

target_link_libraries(pcl_action
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${PCL_LIBRARIES}
)




