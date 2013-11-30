cmake_minimum_required(VERSION 2.8.3)
project(grizzly_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS gazebo_plugins gazebo_ros gazebo_msgs roscpp std_msgs grizzly_msgs)
find_package(gazebo)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES grizzly_gazebo_plugins
  CATKIN_DEPENDS gazebo_plugins gazebo_ros gazebo_msgs roscpp std_msgs
  DEPENDS system_lib
)

## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/grizzly_plugin)
add_dependencies(${PROJECT_NAME} grizzly_msgs_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)  

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
