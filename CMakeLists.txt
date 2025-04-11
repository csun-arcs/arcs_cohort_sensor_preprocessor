cmake_minimum_required(VERSION 3.8)
project(arcs_cohort_sensor_preprocessor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(pointcloud_transformer SHARED
  src/pointcloud_transformer.cpp
)

target_include_directories(pointcloud_transformer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(pointcloud_transformer
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
  pcl_ros
  pcl_conversions
  rclcpp_components
)

rclcpp_components_register_nodes(pointcloud_transformer "arcs_cohort_sensor_preprocessor::PointCloudTransformer")

install(TARGETS pointcloud_transformer
  EXPORT export_pointcloud_transformer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
