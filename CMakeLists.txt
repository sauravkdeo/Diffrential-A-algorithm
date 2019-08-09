cmake_minimum_required(VERSION 2.8.3)
project(frontier_exploration_turtlebot)

## Compile as C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
 find_package(OpenCV 2 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package()

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(frontierExplorer 
	src/main.cpp
    src/coord.cpp
    src/map.cpp
    src/node.cpp
    src/nodeposition.cpp
	src/PathPlanning.cpp
	)
	
target_link_libraries(frontierExplorer
	${catkin_LIBRARIES}
)
target_link_libraries(frontierExplorer ${OpenCV_LIBRARIES})



