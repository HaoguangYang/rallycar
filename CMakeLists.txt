cmake_minimum_required(VERSION 3.8)

# this sets the ${PROJECT_NAME} variable within this script
project(rallycar)

# enable terminal output of all compile-time warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_python REQUIRED)
# automatically add additional dependencies declared in package.xml
ament_auto_find_build_dependencies()

# this creates a shared library from the C++ sources
ament_auto_add_library(rallycar_driver_lib SHARED
  src/serial_impl.cc
  src/serial.cc
  src/rallycar_driver_node.cpp
)

# this plugs the library into the ROS2 component system and turns it into
# a standard ROS2 node "rallycar_driver_node" executable.
rclcpp_components_register_node(rallycar_driver_lib
  PLUGIN "rallycar::RallycarDriverNode"
  EXECUTABLE rallycar_driver_node
)

# Create a Python library from "rallycar/" that others can import with:
# from rallycar import ...​
ament_python_install_package(${PROJECT_NAME})

# install python scripts as python executables​, that others can run with:
# ros2 run rallycar ...
install(PROGRAMS
  scripts/rally_teleop_keyboard.py
  scripts/path_server.py
  scripts/path_recorder.py
  scripts/odom_tf_publisher.py
  scripts/scanmatcher_odom_repub.py
  DESTINATION lib/${PROJECT_NAME}
)

# build rviz plugins -- need to specify qt dependency manually
find_package(Qt5 REQUIRED Core Widgets)
# need to generate MOC files manually to prevent namespace mangling
set(rallycar_rviz_plugin_headers_to_moc
  include/rallycar/remove_last_nav_goal_tool.hpp
  include/rallycar/rviz2_ros2launch.hpp
)
foreach(header "${rallycar_rviz_plugin_headers_to_moc}")
  qt5_wrap_cpp(rallycar_rviz_plugin_moc_files "${header}")
endforeach()

# and ament_auto_add_library will work just as well.
ament_auto_add_library(rallycar_rviz_plugin_lib SHARED
  ${rallycar_rviz_plugin_moc_files}
  src/remove_last_nav_goal_tool.cpp
  src/rviz2_ros2launch.cpp
)
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# tests, beginners may ignore this "if" section
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  # export information to downstream (depended-by) packages
  CONFIG_EXTRAS "${PROJECT_NAME}-extras.cmake"
  # install additional folders
  INSTALL_TO_SHARE resources urdf launch param icons
)
