cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_highlevel_controller)
add_definitions(--std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
	catkin REQUIRED COMPONENTS roscpp rospy sensor_msgs
)

catkin_package(
	INCLUDE_DIRS include
	# LIBRARIES
	CATKIN_DEPENDS roscpp rospy sensor_msgs
	# DEPENDS
)

include_directories(
 include ${catkin_INCLUDE_DIRS}
)

add_executable(
	${PROJECT_NAME} src/turtlebot_laserscan_range/${PROJECT_NAME}_node.cpp src/turtlebot_laserscan_range/TurtlebotHighlevelController.cpp
)

target_link_libraries(
	${PROJECT_NAME} ${catkin_LIBRARIES}
)


