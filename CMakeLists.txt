cmake_minimum_required(VERSION 2.8.3)
project(toyota_localplanner)

set(toyota_localplanner_ROS_DEPS base_local_planner costmap_2d geometry_msgs nav_core nav_msgs pluginlib roscpp std_msgs tf)

find_package(catkin REQUIRED COMPONENTS ${toyota_localplanner_ROS_DEPS})

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LINK_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${toyota_localplanner_ROS_DEPS}
)

add_library(toyota_localplanner src/toyota_localplanner.cpp src/vehicle_control.cpp)
target_link_libraries(toyota_localplanner ${catkin_LIBRARIES})

install(TARGETS toyota_localplanner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
