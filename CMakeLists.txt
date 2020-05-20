cmake_minimum_required(VERSION 2.8.3)
project(dualarm_mobile_bringup)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  nodelet
  tf
)

catkin_package(
  LIBRARIES 
    dualarm_mobile_bringup
  CATKIN_DEPENDS 
    geometry_msgs 
    nav_msgs 
    roscpp 
    std_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

add_executable(bringup_publisher src/bringup_publisher.cpp)
target_link_libraries(bringup_publisher ${catkin_LIBRARIES})

add_executable(bringup_publisher_new src/bringup_publisher_new.cpp)
target_link_libraries(bringup_publisher_new ${catkin_LIBRARIES})

add_executable(odom_publisher src/odom_publisher.cpp)
target_link_libraries(odom_publisher ${catkin_LIBRARIES})

