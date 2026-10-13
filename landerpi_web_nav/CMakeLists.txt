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
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenSSL REQUIRED)

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

add_library(line_extractor src/line_extractor.cpp)
target_include_directories(line_extractor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(line_extractor PUBLIC cxx_std_17)

# Only this one needs OpenCV (findContours); line_extractor stays pure.
add_library(map_contours src/map_contours.cpp)
target_include_directories(map_contours PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(map_contours PUBLIC line_extractor ${OpenCV_LIBS})
target_compile_features(map_contours PUBLIC cxx_std_17)

add_library(ip_filter src/ip_filter.cpp)
target_include_directories(ip_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ip_filter PUBLIC cxx_std_17)

add_library(auth_manager src/auth_manager.cpp)
target_include_directories(auth_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(auth_manager PUBLIC OpenSSL::Crypto)
target_compile_features(auth_manager PUBLIC cxx_std_17)

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

add_library(wall_vectorizer_node_lib src/wall_vectorizer_node.cpp)
target_include_directories(wall_vectorizer_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(wall_vectorizer_node_lib map_contours line_extractor)
ament_target_dependencies(wall_vectorizer_node_lib
  rclcpp nav_msgs std_srvs tf2 tf2_ros geometry_msgs visualization_msgs)

add_executable(wall_vectorizer_node src/wall_vectorizer_node_main.cpp)
target_link_libraries(wall_vectorizer_node wall_vectorizer_node_lib)
ament_target_dependencies(wall_vectorizer_node rclcpp)

install(TARGETS covariance_stamper covariance_stamper_node_lib
  scan_recenter scan_recenter_node_lib line_extractor map_contours
  wall_vectorizer_node_lib ip_filter auth_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS covariance_stamper_node scan_recenter_node wall_vectorizer_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/a7_set_password
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_covariance_stamper test/test_covariance_stamper.cpp)
  target_link_libraries(test_covariance_stamper covariance_stamper)

  ament_add_gtest(test_scan_recenter test/test_scan_recenter.cpp)
  target_link_libraries(test_scan_recenter scan_recenter)

  ament_add_gtest(test_line_extractor test/test_line_extractor.cpp)
  target_link_libraries(test_line_extractor line_extractor)

  ament_add_gtest(test_ip_filter test/test_ip_filter.cpp)
  target_link_libraries(test_ip_filter ip_filter)

  ament_add_gtest(test_auth_manager test/test_auth_manager.cpp)
  target_link_libraries(test_auth_manager auth_manager)

  ament_add_gtest(test_map_contours test/test_map_contours.cpp)
  target_link_libraries(test_map_contours map_contours)
  target_compile_definitions(test_map_contours PRIVATE
    TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
endif()

ament_package()
