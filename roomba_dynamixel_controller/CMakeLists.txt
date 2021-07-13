cmake_minimum_required(VERSION 3.0.2)
project(roomba_dynamixel_controller)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  trajectory_msgs
  dynamixel_angle_msgs
  tf2
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cmake_modules geometry_msgs roscpp sensor_msgs std_msgs std_srvs trajectory_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(roomba_dynamixel_controller src/roomba_dynamixel_controller.cpp)
add_executable(test_dynamixel src/test_dynamixel.cpp)
add_executable(teleop_dynamixel_angle src/teleop_dynamixel_angle.cpp)

add_dependencies(roomba_dynamixel_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(test_dynamixel ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(teleop_dynamixel_angle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(roomba_dynamixel_controller ${catkin_LIBRARIES})
target_link_libraries(test_dynamixel ${catkin_LIBRARIES})
target_link_libraries(teleop_dynamixel_angle ${catkin_LIBRARIES})
