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
  # TODO: re-enable linters once existing lint errors are resolved.
  # find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  # set(ament_cmake_copyright_FOUND TRUE)
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()

  option(
    RUN_AUTODOCKING_TESTS
    "Include hardware-in-the-loop autodocking tests in CTest (requires physical charging dock)"
    OFF
  )

  ament_add_pytest_test(
    test_docking_server_unit
    test/unit/test_docking_server_unit.py
    TIMEOUT 60
  )
  if(TEST test_docking_server_unit)
    set_tests_properties(test_docking_server_unit PROPERTIES LABELS "pytest;unit")
  endif()

  if(RUN_AUTODOCKING_TESTS)
    ament_add_pytest_test(
      test_autodocking
      test/system/test_autodocking.py
      TIMEOUT 240
    )
    if(TEST test_autodocking)
      set_tests_properties(test_autodocking PROPERTIES LABELS "pytest;hardware;autodocking")
    endif()
  endif()
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
