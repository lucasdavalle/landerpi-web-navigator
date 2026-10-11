cmake_minimum_required(VERSION 3.8)
project(landerpi_web_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Pure logic, no rclcpp dependency: own target so it can be gtest-ed without
# bringing up rclcpp.
add_library(covariance_stamper src/covariance_stamper.cpp)
target_include_directories(covariance_stamper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(covariance_stamper PUBLIC cxx_std_17)

add_library(scan_recenter src/scan_recenter.cpp)
target_include_directories(scan_recenter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(scan_recenter PUBLIC cxx_std_17)

add_library(covariance_stamper_node_lib src/covariance_stamper_node.cpp)
target_include_directories(covariance_stamper_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(covariance_stamper_node_lib covariance_stamper)
ament_target_dependencies(covariance_stamper_node_lib rclcpp nav_msgs sensor_msgs std_srvs)

add_executable(covariance_stamper_node src/covariance_stamper_node_main.cpp)
target_link_libraries(covariance_stamper_node covariance_stamper_node_lib)
ament_target_dependencies(covariance_stamper_node rclcpp)

add_library(scan_recenter_node_lib src/scan_recenter_node.cpp)
target_include_directories(scan_recenter_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(scan_recenter_node_lib scan_recenter)
ament_target_dependencies(scan_recenter_node_lib rclcpp sensor_msgs std_srvs)

add_executable(scan_recenter_node src/scan_recenter_node_main.cpp)
target_link_libraries(scan_recenter_node scan_recenter_node_lib)
ament_target_dependencies(scan_recenter_node rclcpp)

install(TARGETS covariance_stamper covariance_stamper_node_lib
  scan_recenter scan_recenter_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS covariance_stamper_node scan_recenter_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_covariance_stamper test/test_covariance_stamper.cpp)
  target_link_libraries(test_covariance_stamper covariance_stamper)

  ament_add_gtest(test_scan_recenter test/test_scan_recenter.cpp)
  target_link_libraries(test_scan_recenter scan_recenter)
endif()

ament_package()
