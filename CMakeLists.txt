cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_highlevel_controller)
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
  catkin REQUIRED COMPONENTS roscpp rospy sensor_msgs tf actionlib_msgs actionlib
)
find_package(Boost REQUIRED COMPONENTS system)

## Find system libraries
#find_package(Eigen3 REQUIRED)
#find_package(Boost REQUIRED)

add_message_files(
  FILES
  Target.msg
)

add_action_files(
  DIRECTORY action
  FILES controller.action Fibonacci.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_core
  CATKIN_DEPENDS roscpp rospy sensor_msgs actionlib_msgs
  # DEPENDS
)

#
#  BUILD  #
#


## Declare a cpp library
add_library(
  ${PROJECT_NAME}_core	
  src/turtlebot_laserscan_range/Algorithm.cpp
)


include_directories(
  include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

########### TurtlebotHighlevelController ###############
add_executable(
  ${PROJECT_NAME} src/turtlebot_laserscan_range/${PROJECT_NAME}_node.cpp	
  src/turtlebot_laserscan_range/TurtlebotHighlevelController.cpp
)

add_dependencies(${PROJECT_NAME}_core
  ${catkin_EXPORTED_TARGETS}
)


#
add_dependencies(${PROJECT_NAME} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_core	
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)



########### turtlebot_mapping_node ###############
add_executable(turtlebot_mapping_node  
  src/turtlebot_mapping/TurtlebotMapping.cpp
  src/turtlebot_mapping/turtlebot_mapping_node.cpp
)

add_library(turtlebot_mapping
  src/turtlebot_mapping/TurtlebotMapping.cpp
)

add_dependencies(turtlebot_mapping 
	turtlebot_highlevel_controller_generate_messages_cpp
)

target_link_libraries(turtlebot_mapping
  ${catkin_LIBRARIES}
)

target_link_libraries(turtlebot_mapping_node 
  turtlebot_mapping
  ${catkin_LIBRARIES}
)

########### fibonacci_client ###############
add_executable(mapping_client src/mapping_client.cpp)

target_link_libraries(
  mapping_client
  ${catkin_LIBRARIES}
)

add_dependencies(
  mapping_client
  # The one below is the name of your package!!
  ${catkin_EXPORTED_TARGETS}
)

#
#  INSTALL
#

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(
  DIRECTORY doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
