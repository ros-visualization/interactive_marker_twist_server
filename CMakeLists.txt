cmake_minimum_required(VERSION 2.8.3)

project(interactive_marker_twist_server)

find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  roscpp
  roslaunch
  roslint
  visualization_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/marker_server)

add_executable(${PROJECT_NAME}_node src/marker_server_node)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_node
  PROPERTIES OUTPUT_NAME marker_server PREFIX "")

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  roslaunch_add_file_check(launch/interactive_markers.launch
    DEPENDENCIES ${PROJECT_NAME}_node)
  roslint_cpp()
  roslint_add_test()
endif()
