cmake_minimum_required(VERSION 3.8)
project(gazebo_ackermann_steering_vehicle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Waypoint.msg"
  "msg/Path.msg"
  DEPENDENCIES std_msgs
)

include_directories(include ${EIGEN3_INCLUDE_DIR})

add_executable(vehicle_controller src/vehicle_controller.cpp)
ament_target_dependencies(vehicle_controller rclcpp std_msgs)

add_executable(joystick_controller src/joystick_controller.cpp)
ament_target_dependencies(joystick_controller rclcpp std_msgs sensor_msgs)

# Gazebo TF broadcaster
add_executable(gazebo_tf_broadcaster src/gazebo_tf_broadcaster.cpp)
ament_target_dependencies(gazebo_tf_broadcaster rclcpp nav_msgs geometry_msgs tf2_ros)

# Path follower node with all controllers
add_executable(path_follower_node
  src/path_follower_node.cpp
  src/path_manager.cpp
  src/state_estimator.cpp
  src/trajectory_generator.cpp
  src/metrics_logger.cpp
  src/pid_controller.cpp
  src/stanley_controller.cpp
  src/lqr_controller.cpp
  src/sliding_mode_controller.cpp
)
ament_target_dependencies(path_follower_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(path_follower_node Eigen3::Eigen)

# Link against custom messages
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(path_follower_node "${cpp_typesupport_target}")

install(
  DIRECTORY launch model config scripts
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/circle_motion.py
    scripts/odometry_to_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  vehicle_controller
  joystick_controller
  path_follower_node
  gazebo_tf_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

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

ament_package()
