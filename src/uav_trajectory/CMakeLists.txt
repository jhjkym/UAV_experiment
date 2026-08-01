cmake_minimum_required(VERSION 3.0.2)
project(uav_trajectory)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rostest
  std_msgs
  uav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES uav_trajectory_sampler uav_dynamic_trajectory_generator
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs uav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(uav_trajectory_sampler
  src/trajectory_sampler.cpp
)
target_link_libraries(uav_trajectory_sampler
  ${catkin_LIBRARIES}
)
add_dependencies(uav_trajectory_sampler
  ${catkin_EXPORTED_TARGETS}
)

add_library(uav_dynamic_trajectory_generator
  src/dynamic_trajectory_generator.cpp
)
target_link_libraries(uav_dynamic_trajectory_generator
  ${catkin_LIBRARIES}
)
add_dependencies(uav_dynamic_trajectory_generator
  ${catkin_EXPORTED_TARGETS}
)

add_executable(trajectory_preview_node
  src/trajectory_preview_node.cpp
)
target_link_libraries(trajectory_preview_node
  uav_trajectory_sampler
  ${catkin_LIBRARIES}
)
add_dependencies(trajectory_preview_node
  ${catkin_EXPORTED_TARGETS}
)

add_executable(dynamic_trajectory_publisher_node
  src/dynamic_trajectory_publisher_node.cpp
)
target_link_libraries(dynamic_trajectory_publisher_node
  uav_dynamic_trajectory_generator
  ${catkin_LIBRARIES}
)
add_dependencies(dynamic_trajectory_publisher_node
  ${catkin_EXPORTED_TARGETS}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_trajectory_sampler
    test/test_trajectory_sampler.cpp
  )
  if(TARGET test_trajectory_sampler)
    target_link_libraries(test_trajectory_sampler
      uav_trajectory_sampler
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_dynamic_trajectory_generator
    test/test_dynamic_trajectory_generator.cpp
  )
  if(TARGET test_dynamic_trajectory_generator)
    target_link_libraries(test_dynamic_trajectory_generator
      uav_dynamic_trajectory_generator
      ${catkin_LIBRARIES}
    )
  endif()

  add_rostest(test/trajectory_preview_integration.test)
  add_rostest(test/dynamic_trajectory_preview_integration.test)
  catkin_add_nosetests(../../scripts/analysis/test_trajectory_tracking_metrics.py)
  catkin_add_nosetests(../../scripts/experiments/test_m0_c5b1_protocol.py)
endif()

install(TARGETS
  dynamic_trajectory_publisher_node
  trajectory_preview_node
  uav_dynamic_trajectory_generator
  uav_trajectory_sampler
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
