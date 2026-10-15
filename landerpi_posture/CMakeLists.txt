cmake_minimum_required(VERSION 3.8)
project(landerpi_posture)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(servo_controller_msgs REQUIRED)
find_package(landerpi_nav2_depth_layer REQUIRED)

add_library(posture src/posture.cpp)
target_include_directories(posture PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(posture PUBLIC cxx_std_17)

add_executable(posture_manager_node
  src/posture_manager_node.cpp src/posture_manager_node_main.cpp)
target_include_directories(posture_manager_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_include_directories(posture_manager_node SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(posture_manager_node posture)
ament_target_dependencies(posture_manager_node
  rclcpp rclcpp_lifecycle lifecycle_msgs geometry_msgs sensor_msgs std_msgs std_srvs
  tf2 tf2_ros tf2_eigen servo_controller_msgs landerpi_nav2_depth_layer)

install(TARGETS posture ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS posture_manager_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_posture test/test_posture.cpp)
  target_link_libraries(test_posture posture)
endif()

ament_package()
