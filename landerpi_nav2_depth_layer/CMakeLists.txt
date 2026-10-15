cmake_minimum_required(VERSION 3.8)
project(landerpi_nav2_depth_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# Pure geometry (no ROS): depth -> points, floor calibration, height classes.
add_library(depth_geometry
  src/depth_projector.cpp
  src/floor_calibrator.cpp
  src/height_classifier.cpp)
target_include_directories(depth_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(depth_geometry SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})
target_compile_features(depth_geometry PUBLIC cxx_std_17)

install(TARGETS depth_geometry
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_depth_geometry test/test_depth_geometry.cpp)
  target_link_libraries(test_depth_geometry depth_geometry)
endif()

ament_package()
