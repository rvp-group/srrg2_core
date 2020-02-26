add_executable(test_bag_creator test_bag_creator.cpp)
target_link_libraries(test_bag_creator
  ${catkin_LIBRARIES}
  )

add_executable(test_conversion_in_place test_conversion_in_place.cpp)
target_link_libraries(test_conversion_in_place
  srrg2_system_utils_library
  srrg2_converters_library
  ${catkin_LIBRARIES}
  )

add_executable(test_tf_listener_playback test_tf_listener_playback.cpp)
target_link_libraries(test_tf_listener_playback
  srrg2_system_utils_library
  srrg2_converters_library
  ${catkin_LIBRARIES}
  )

add_executable(test_tf_listener_node test_tf_listener_node.cpp)
target_link_libraries(test_tf_listener_node
  srrg2_system_utils_library
  srrg2_converters_library
  ${catkin_LIBRARIES}
  )

add_executable(test_generic_listener_node test_generic_listener_node.cpp)
target_link_libraries(test_generic_listener_node
  srrg2_system_utils_library
  srrg2_converters_library
  ${catkin_LIBRARIES}
  )

add_executable(test_get_all_topics test_get_all_topics.cpp)
target_link_libraries(test_get_all_topics
  srrg2_system_utils_library
  srrg2_converters_library
  ${catkin_LIBRARIES}
  )

add_executable(test_message_source_ros test_message_source_ros.cpp)
target_link_libraries(test_message_source_ros
  srrg2_messages_ros_library
  srrg2_system_utils_library
  ${catkin_LIBRARIES}
  )

add_executable(test_tf_publisher_node test_tf_publisher_node.cpp)
target_link_libraries(test_tf_publisher_node
  srrg2_messages_ros_library
  ${catkin_LIBRARIES}
  )

#ia goddamn platform tests  
add_executable(test_tf_from_source test_tf_from_source.cpp)
target_link_libraries(test_tf_from_source
  srrg2_messages_ros_library
  srrg2_system_utils_library
  ${catkin_LIBRARIES})
  
add_executable(test_message_source_platform test_message_source_platform.cpp)
target_link_libraries(test_message_source_platform
  srrg2_messages_ros_library
  srrg2_system_utils_library
  ${catkin_LIBRARIES})
  
#ia viewer stuff
# add_executable(test_rviz test_rviz.cpp)
# target_link_libraries(test_rviz
#   srrg2_messages_ros_library
#   ${catkin_LIBRARIES})
  
add_executable(test_buffer_sink_ros test_buffer_sink_ros.cpp)
target_link_libraries(test_buffer_sink_ros
  srrg2_viewer_core_ros_library
  ${catkin_LIBRARIES})
  
add_executable(test_buffer_source_ros test_buffer_source_ros.cpp)
target_link_libraries(test_buffer_source_ros
  srrg2_viewer_core_ros_library
  ${catkin_LIBRARIES})
  
add_executable(test_buffer_message test_buffer_message.cpp)
target_link_libraries(test_buffer_message
  srrg2_viewer_core_ros_library
  ${catkin_LIBRARIES})
