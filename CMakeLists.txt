cmake_minimum_required(VERSION 3.0.2)
project(3D_segmentation_model)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_filters
  tf
  pcl_ros
  cv_bridge
)

## System dependencies are found with CMake's conventions
find_package(PCL 1.3 REQUIRED)


catkin_package(INCLUDE_DIRS include
#  LIBRARIES 3D_segmentation_model
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(filter_cloud src/filter_cloud.cpp)
target_link_libraries(filter_cloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(build_dataset src/build_dataset.cpp)
target_link_libraries(build_dataset ${catkin_LIBRARIES} ${PCL_LIBRARIES})
