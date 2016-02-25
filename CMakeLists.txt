cmake_minimum_required(VERSION 2.8.3)
project(velodyne_detect_person)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  genmsg
  PCL REQUIRED
)

add_message_files(FILES pointCloudVector.msg)
generate_messages(DEPENDENCIES sensor_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(extract_clusters src/extract_clusters.cpp)
target_link_libraries(extract_clusters ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(extract_clusters velodyne_detect_person_generate_messages_cpp)
