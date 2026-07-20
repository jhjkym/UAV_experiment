cmake_minimum_required(VERSION 3.0.2)
project(uav_px4_bridge)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  roscpp
  sensor_msgs
  uav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES uav_px4_bridge_frame_conversions uav_px4_bridge_state_bridge
  CATKIN_DEPENDS geometry_msgs mavros_msgs nav_msgs roscpp sensor_msgs uav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(uav_px4_bridge_frame_conversions
  src/frame_conversions.cpp
)
target_link_libraries(uav_px4_bridge_frame_conversions
  ${catkin_LIBRARIES}
)

add_library(uav_px4_bridge_state_bridge
  src/state_bridge.cpp
)
target_link_libraries(uav_px4_bridge_state_bridge
  ${catkin_LIBRARIES}
)
add_dependencies(uav_px4_bridge_state_bridge
  ${catkin_EXPORTED_TARGETS}
)

add_executable(mavros_state_bridge_node
  src/mavros_state_bridge_node.cpp
)
target_link_libraries(mavros_state_bridge_node
  uav_px4_bridge_state_bridge
  ${catkin_LIBRARIES}
)
add_dependencies(mavros_state_bridge_node
  ${catkin_EXPORTED_TARGETS}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  catkin_add_gtest(test_frame_conversions
    test/test_frame_conversions.cpp
  )
  if(TARGET test_frame_conversions)
    target_link_libraries(test_frame_conversions
      uav_px4_bridge_frame_conversions
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_state_bridge
    test/test_state_bridge.cpp
  )
  if(TARGET test_state_bridge)
    target_link_libraries(test_state_bridge
      uav_px4_bridge_state_bridge
      ${catkin_LIBRARIES}
    )
  endif()

  add_rostest(test/state_bridge_integration.test)
endif()

install(TARGETS
  mavros_state_bridge_node
  uav_px4_bridge_frame_conversions
  uav_px4_bridge_state_bridge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
