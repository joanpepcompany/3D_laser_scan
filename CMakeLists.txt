cmake_minimum_required(VERSION 2.8.3)
project(3D_laser_scan)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_conversions
  pcl_ros
  std_msgs
  sensor_msgs
  tf
  laser_geometry
  std_srvs
)

# find_package(PCL 1.7 REQUIRED COMPONENTS)

catkin_package( 
  CATKIN_DEPENDS 
  LIBRARIES 
  roscpp 
  INCLUDE_DIRS include
  LIBRARIES nodelet 
  sensor_msgs 
  std_msgs
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  # ${PCL_INCLUDE_DIRS}
)
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

# link_directories(${PCL_LIBRARY_DIRS})

add_executable(${PROJECT_NAME}
  src/3D_laser_scan.cpp 
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES})
