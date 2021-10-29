cmake_minimum_required(VERSION 3.5)

project(interactive_marker_twist_server)

find_package(ament_cmake REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(${PROJECT_NAME} src/marker_server.cpp)
ament_target_dependencies(${PROJECT_NAME}
  interactive_markers
  rclcpp
  tf2
  visualization_msgs
  geometry_msgs)

# Not sure what this does
set_target_properties(${PROJECT_NAME}
  PROPERTIES OUTPUT_NAME marker_server PREFIX "")

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)

#if(CATKIN_ENABLE_TESTING)
#  find_package(roslaunch REQUIRED)
#  roslaunch_add_file_check(launch/interactive_markers.launch)
#
#  find_package(roslint REQUIRED)
#  roslint_cpp()
#  roslint_add_test()
#endif()

ament_package()