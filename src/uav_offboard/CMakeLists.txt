cmake_minimum_required(VERSION 3.0.2)
project(uav_offboard)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  rostest
  std_msgs
  std_srvs
  uav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES uav_offboard_adapter
  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp std_msgs std_srvs uav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(uav_offboard_adapter
  src/offboard_adapter.cpp
)
target_link_libraries(uav_offboard_adapter
  ${catkin_LIBRARIES}
)
add_dependencies(uav_offboard_adapter
  ${catkin_EXPORTED_TARGETS}
)

add_executable(offboard_adapter_node
  src/offboard_adapter_node.cpp
)
target_link_libraries(offboard_adapter_node
  uav_offboard_adapter
  ${catkin_LIBRARIES}
)
add_dependencies(offboard_adapter_node
  ${catkin_EXPORTED_TARGETS}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_offboard_adapter
    test/test_offboard_adapter.cpp
  )
  if(TARGET test_offboard_adapter)
    target_link_libraries(test_offboard_adapter
      uav_offboard_adapter
      ${catkin_LIBRARIES}
    )
  endif()

  add_rostest(test/offboard_adapter_dry_run.test)
  add_rostest(test/offboard_adapter_output_gate.test)
endif()

install(TARGETS
  offboard_adapter_node
  uav_offboard_adapter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
