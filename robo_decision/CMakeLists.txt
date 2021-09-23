cmake_minimum_required(VERSION 3.1)
project(robo_decision)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
  geometry_msgs
  actionlib
  actionlib_msgs
  move_base_msgs
  std_msgs
  )
  
include_directories(${catkin_INCLUDE_DIRS})
include_directories( ${PROJECT_SOURCE_DIR}
   ${RPOJECT_SOURCE_DIR}/include)

catkin_package(
  #  LIBRAIRES
  #  robo_planning
  )

add_library(robo_decision_libs
  src/${PROJECT_NAME}.cc
)

target_link_libraries(robo_decision_libs
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node 
    src/robo_decision_node.cc
)

target_link_libraries(${PROJECT_NAME}_node 
  ${catkin_LIBRARIES}
  robo_decision_libs
)
