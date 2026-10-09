cmake_minimum_required(VERSION 3.8)
project(stretch_core)

# Set default build type to Release if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
endif()

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
find_package(diagnostic_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenMP REQUIRED)

set(lidar_filter_dependencies
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_eigen
  tf2_sensor_msgs
  message_filters
)

include_directories(${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(lidar_filter_lib_sources
  src/robot_self_filter.cpp
  src/sor_filter.cpp
  src/floor_plane_filter.cpp
  src/dual_lidar_pipeline.cpp
)

add_executable(dual_lidar_laserscan
  src/dual_lidar_laserscan_node.cpp
  ${lidar_filter_lib_sources})
ament_target_dependencies(dual_lidar_laserscan ${lidar_filter_dependencies} pcl_conversions diagnostic_msgs)
target_include_directories(dual_lidar_laserscan PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(dual_lidar_laserscan ${PCL_LIBRARIES})
if(OpenMP_CXX_FOUND)
  target_link_libraries(dual_lidar_laserscan OpenMP::OpenMP_CXX)
endif()

add_executable(dual_lidar_pointcloud_merger
  src/dual_lidar_pointcloud_merger_node.cpp)
target_link_libraries(dual_lidar_pointcloud_merger OpenMP::OpenMP_CXX)
ament_target_dependencies(dual_lidar_pointcloud_merger ${lidar_filter_dependencies})
target_include_directories(dual_lidar_pointcloud_merger PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)


add_executable(robot_footprint_publisher
  src/robot_footprint_publisher_node.cpp
  src/robot_self_filter.cpp)
ament_target_dependencies(
  robot_footprint_publisher
  rclcpp
  geometry_msgs
  sensor_msgs
  std_srvs
  visualization_msgs
  tf2_ros
  tf2_eigen)
target_include_directories(robot_footprint_publisher PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

install(TARGETS
  dual_lidar_laserscan
  dual_lidar_pointcloud_merger
  robot_footprint_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(
  FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
