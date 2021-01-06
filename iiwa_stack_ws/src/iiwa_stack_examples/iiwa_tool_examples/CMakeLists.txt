cmake_minimum_required(VERSION 2.8.3)
project(iiwa_tool_examples)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  #iiwa_ros
  iiwa_msgs#added
  geometry_msgs#added
  moveit_core 
  moveit_ros_planning
  moveit_ros_planning_interface
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)#added



add_executable(command_robot_moveit src/command_robot_moveit.cpp include/numofmesh.h)
add_dependencies(command_robot_moveit iiwa_msgs_generate_messages_cpp)
target_link_libraries(command_robot_moveit ${catkin_LIBRARIES})

#belows are added
add_executable(rcvMappingData src/rcvMappingData.cpp include/numofmesh.h)
add_dependencies(rcvMappingData iiwa_msgs_generate_messages_cpp)
target_link_libraries(rcvMappingData ${catkin_LIBRARIES})

add_executable(rcvNewPose src/rcvNewPose.cpp)
add_dependencies(rcvNewPose iiwa_msgs_generate_messages_cpp)
target_link_libraries(rcvNewPose ${catkin_LIBRARIES})

add_executable(rcvARPose src/rcvARPose.cpp)
add_dependencies(rcvARPose iiwa_msgs_generate_messages_cpp)
target_link_libraries(rcvARPose ${catkin_LIBRARIES})

add_executable(rcvMoveCommand src/rcvMoveCommand.cpp)
add_dependencies(rcvMoveCommand iiwa_msgs_generate_messages_cpp)
target_link_libraries(rcvMoveCommand ${catkin_LIBRARIES})
