cmake_minimum_required(VERSION 2.8.3)
project(box_detector)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(point_cloud_publish src/point_cloud_publish.cpp)
add_executable(point_cloud_subscriber src/point_cloud_subscriber.cpp)

target_link_libraries(point_cloud_publish ${catkin_LIBRARIES})
target_link_libraries(point_cloud_subscriber ${catkin_LIBRARIES})
