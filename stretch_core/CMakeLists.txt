cmake_minimum_required(VERSION 3.8)
project(stretch_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenMP REQUIRED)

set(lidar_filter_dependencies
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_eigen
  tf2_sensor_msgs
  message_filters
)

include_directories(${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

macro(add_dual_lidar_filter_executable target_name source_file)
  add_executable(${target_name} ${source_file})
  ament_target_dependencies(${target_name} ${lidar_filter_dependencies} pcl_conversions)
  target_include_directories(${target_name} PUBLIC ${EIGEN3_INCLUDE_DIRS})
  target_link_libraries(${target_name} ${PCL_LIBRARIES})
  if(OpenMP_CXX_FOUND)
    target_link_libraries(${target_name} OpenMP::OpenMP_CXX)
  endif()
endmacro()

add_executable(region_dual_lidar_laserscan src/region_dual_lidar_laserscan.cpp)
ament_target_dependencies(region_dual_lidar_laserscan ${lidar_filter_dependencies} pcl_conversions)
target_include_directories(region_dual_lidar_laserscan PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(region_dual_lidar_laserscan ${PCL_LIBRARIES})

add_dual_lidar_filter_executable(voxel_dual_lidar_laserscan src/voxel_dual_lidar_laserscan.cpp)
add_dual_lidar_filter_executable(voxel_dual_lidar_laserscan_RANSAC src/voxel_dual_lidar_laserscan_RANSAC.cpp)

install(TARGETS
  region_dual_lidar_laserscan
  voxel_dual_lidar_laserscan
  voxel_dual_lidar_laserscan_RANSAC
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(
  FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
