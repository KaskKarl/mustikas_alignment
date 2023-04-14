cmake_minimum_required(VERSION 3.0.2)
project(mustikas_alignment)

 #Compile as C++17, supported in ROS Kinetic and newer
 add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(DarkHelp_DIR "/home/peko/DarkHelp")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  sensor_msgs
  moveit_ros_planning_interface
  message_filters
  std_srvs
  std_msgs
  message_generation
  
)

 find_package(OpenCV REQUIRED)
 find_package(PkgConfig REQUIRED)
 pkg_check_modules( DarkNet REQUIRED DarkNet )
 pkg_check_modules( DarkHelp REQUIRED DarkHelp )
 
add_message_files(
 FILES
 ROIs.msg
 PointArray.msg
)
 
add_service_files( 
 FILES
 Control.srv
 MoveToPoint.srv
) 



generate_messages(
 DEPENDENCIES
 geometry_msgs
 sensor_msgs
)
 
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
#  LIBRARIES mustikas_alignment
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp sensor_msgs moveit_ros_planning_interface message_filters std_srvs std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/mustikas_alignment.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/mustikas_alignment_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mustikas_alignment.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_executable(detection_node src/detection_node.cpp)
target_link_libraries(detection_node ${catkin_LIBRARIES} ${OpenCV_INCLUDE_DIRS} ${DarkNet_LIBRARIES} ${DarkHelp_LIBRARIES} opencv_imgcodecs opencv_dnn darknet)

add_executable(location_node src/location_node.cpp)
target_link_libraries(location_node ${catkin_LIBRARIES})

add_executable(coordinate_translator src/coordinate_translator.cpp)
target_link_libraries(coordinate_translator ${catkin_LIBRARIES})

add_executable(absolute_move src/absolute_move.cpp)
target_link_libraries(absolute_move ${catkin_LIBRARIES})

add_executable(location_node_calc src/location_node_calc.cpp)
target_link_libraries(location_node_calc ${catkin_LIBRARIES} ${OpenCV_INCLUDE_DIRS} opencv_imgcodecs opencv_dnn)

add_executable(absolute_move_cartesian src/absolute_move_cartesian.cpp)
target_link_libraries(absolute_move_cartesian ${catkin_LIBRARIES})

add_executable(xarm_control_node src/xarm_control_node.cpp)
target_link_libraries(xarm_control_node ${catkin_LIBRARIES})

add_executable(absolute_move_server src/absolute_move_server.cpp)
target_link_libraries(absolute_move_server ${catkin_LIBRARIES})

add_executable(ready_move_server src/ready_move_server.cpp)
target_link_libraries(ready_move_server ${catkin_LIBRARIES})

add_executable(xarm_control_node2 src/xarm_control_node2.cpp)
target_link_libraries(xarm_control_node2 ${catkin_LIBRARIES})

add_executable(2detection_node src/2detection_node.cpp)
target_link_libraries(2detection_node ${catkin_LIBRARIES} ${OpenCV_INCLUDE_DIRS} ${DarkNet_LIBRARIES} ${DarkHelp_LIBRARIES} opencv_imgcodecs opencv_dnn darknet)

add_executable(2location_node_calc src/2location_node_calc.cpp)
target_link_libraries(2location_node_calc ${catkin_LIBRARIES})

add_executable(2location_node src/2location_node.cpp)
target_link_libraries(2location_node ${catkin_LIBRARIES})

add_executable(2coordinate_translator src/2coordinate_translator.cpp)
target_link_libraries(2coordinate_translator ${catkin_LIBRARIES})

add_executable(commander_node src/commander_node.cpp)
target_link_libraries(commander_node ${catkin_LIBRARIES})

add_executable(2xarm_control_node2 src/2xarm_control_node2.cpp)
target_link_libraries(2xarm_control_node2 ${catkin_LIBRARIES})
