cmake_minimum_required(VERSION 3.8)
project(landerpi_nav2_depth_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

# Pure geometry (no ROS): depth -> points, floor calibration, height classes.
add_library(depth_geometry
  src/depth_projector.cpp
  src/floor_calibrator.cpp
  src/height_classifier.cpp
  src/observation_cloud.cpp
  src/clearance_check.cpp
  src/crouch_grid.cpp)
target_include_directories(depth_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(depth_geometry SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})
target_compile_features(depth_geometry PUBLIC cxx_std_17)

# The Nav2 costmap plugin (loaded by pluginlib, hence a shared library).
add_library(depth_obstacle_layer SHARED src/depth_obstacle_layer.cpp)
target_link_libraries(depth_obstacle_layer depth_geometry)
ament_target_dependencies(depth_obstacle_layer
  rclcpp nav2_costmap_2d pluginlib sensor_msgs std_msgs tf2 tf2_ros tf2_eigen)
pluginlib_export_plugin_description_file(nav2_costmap_2d plugins.xml)

install(TARGETS depth_geometry depth_obstacle_layer
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_depth_geometry test/test_depth_geometry.cpp)
  target_link_libraries(test_depth_geometry depth_geometry)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3 rclcpp nav2_costmap_2d pluginlib sensor_msgs std_msgs tf2 tf2_ros tf2_eigen)

ament_package()
