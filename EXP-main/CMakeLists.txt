cmake_minimum_required(VERSION 3.0.2)
project(fastlio_bridge)

add_compile_options(-std=c++14 -O3 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  diagnostic_msgs
  message_filters
  message_generation
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)
find_package(PCL REQUIRED COMPONENTS common filters kdtree search segmentation)
find_package(Eigen3 REQUIRED)

add_message_files(FILES
  DynamicObject.msg
  DynamicObjectArray.msg
  SemanticDetection2D.msg
  SemanticDetection2DArray.msg
)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  CATKIN_DEPENDS
    diagnostic_msgs geometry_msgs message_filters message_runtime nav_msgs pcl_conversions
    pcl_ros roscpp sensor_msgs std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros
    visualization_msgs
  DEPENDS EIGEN3 PCL
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

function(add_fastlio_node target source)
  add_executable(${target} src/${source})
  add_dependencies(${target} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${target} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
endfunction()

add_fastlio_node(fastlio_bridge_node fastlio_bridge.cpp)
add_fastlio_node(cloud_bridge_node cloud_bridge.cpp)
add_fastlio_node(dynamic_detector_node dynamic_detector.cpp)
add_fastlio_node(dynamic_cluster_node dynamic_cluster.cpp)
add_fastlio_node(dynamic_predictor_node dynamic_predictor.cpp)
add_fastlio_node(dynamic_marker_node dynamic_marker.cpp)
add_fastlio_node(collision_risk_node collision_risk.cpp)
add_fastlio_node(static_collision_risk_node static_collision_risk.cpp)
add_fastlio_node(avoidance_fusion_node avoidance_fusion.cpp)
add_fastlio_node(static_map_builder_node static_map_builder.cpp)
add_fastlio_node(system_watchdog_node system_watchdog.cpp)
add_fastlio_node(semantic_fusion_node semantic_fusion.cpp)

# Prometheus is optional so the perception package can still compile and be
# replay-tested on a machine that does not have Prometheus installed.
find_package(prometheus_msgs QUIET)
if(prometheus_msgs_FOUND)
  include_directories(${prometheus_msgs_INCLUDE_DIRS})
  add_executable(avoidance_command_bridge_node src/avoidance_command_bridge.cpp)
  add_executable(goal_navigation_node src/goal_navigation.cpp)
  foreach(target avoidance_command_bridge_node goal_navigation_node)
    add_dependencies(${target} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(${target} ${catkin_LIBRARIES} ${prometheus_msgs_LIBRARIES})
  endforeach()
  set(HAVE_PROMETHEUS TRUE)
else()
  message(WARNING "prometheus_msgs not found: command bridge and goal navigation will not be built")
endif()

install(TARGETS
  fastlio_bridge_node cloud_bridge_node dynamic_detector_node
  dynamic_cluster_node dynamic_predictor_node dynamic_marker_node
  collision_risk_node static_collision_risk_node avoidance_fusion_node
  static_map_builder_node system_watchdog_node semantic_fusion_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
if(prometheus_msgs_FOUND)
  install(TARGETS avoidance_command_bridge_node goal_navigation_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()
install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS
  scripts/validate_project.py
  test/synthetic_scene.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
