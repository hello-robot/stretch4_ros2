cmake_minimum_required(VERSION 3.8)
project(stretch_nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# include Cpp "include" directory
include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY stretch_nav2 DESTINATION share/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  examples/dock_nav_cycle.py
  stretch_nav2/switch_controller_config.py
  stretch_nav2/binary_filter_switch.py
  stretch_nav2/global_plan_demo.py
  stretch_nav2/gripper_interceptor.py
  stretch_nav2/aruco_tag_localization.py
  stretch_nav2/calibrate_tag_cli.py
  stretch_nav2/dock_database.py
  stretch_nav2/discover_dock.py
  stretch_nav2/docking_server.py
  stretch_nav2/undocking_server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
