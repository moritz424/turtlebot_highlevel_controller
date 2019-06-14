cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_highlevel_controller)
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
  catkin REQUIRED COMPONENTS roscpp rospy sensor_msgs tf std_msgs message_generation
)

## Find system libraries
#find_package(Eigen3 REQUIRED)
#find_package(Boost REQUIRED)

add_message_files(
  FILES
  Target.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} turtlebot_mapping
  CATKIN_DEPENDS roscpp rospy sensor_msgs message_runtime
  # DEPENDS
)


#
#  BUILD  ##
#

include_directories(
  include ${catkin_INCLUDE_DIRS} turtlebot_mapping
)


## Declare a cpp library

add_library(${PROJECT_NAME}
  src/turtlebot_laserscan_range/Algorithm.cpp

)
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node
  src/turtlebot_laserscan_range/${PROJECT_NAME}_node.cpp  
  src/turtlebot_laserscan_range/TurtlebotHighlevelController.cpp
  
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)



add_library(turtlebot_mapping
  src/turtlebot_mapping/TurtlebotMapping.cpp
)
target_link_libraries(turtlebot_mapping
  ${catkin_LIBRARIES}
)


add_executable(turtlebot_mapping_node  
  src/turtlebot_mapping/TurtlebotMapping.cpp
  src/turtlebot_mapping/turtlebot_mapping_node.cpp
  )
target_link_libraries(turtlebot_mapping_node 
  turtlebot_mapping
  ${catkin_LIBRARIES}
)



add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(turtlebot_mapping
  ${catkin_EXPORTED_TARGETS}
)



#
#  INSTALL
#

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS turtlebot_mapping_node turtlebot_mapping
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

install(
  DIRECTORY include/turtlebot_mapping/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)


# Mark other files for installation
install(
  DIRECTORY doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
