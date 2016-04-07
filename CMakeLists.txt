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

add_executable(set_background src/set_background.cpp)
target_link_libraries(set_background ${catkin_LIBRARIES})
add_dependencies(set_background velodyne_detect_person_generate_messages_cpp)

add_executable(find_person src/find_person.cpp)
target_link_libraries(find_person ${catkin_LIBRARIES})
add_dependencies(find_person velodyne_detect_person_generate_messages_cpp)

add_executable(tf_publisher src/tf_broadcaster.cpp)
target_link_libraries(tf_publisher ${catkin_LIBRARIES})
add_dependencies(tf_publisher velodyne_detect_person_generate_messages_cpp)
