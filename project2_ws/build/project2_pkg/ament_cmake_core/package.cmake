set(_AMENT_PACKAGE_NAME "project2_pkg")
set(project2_pkg_VERSION "0.0.0")
set(project2_pkg_MAINTAINER "leoo <lekangtu@gmail.com>")
set(project2_pkg_BUILD_DEPENDS "rosidl_default_generators" "xacro" "rclpy" "rclcpp" "gazebo_ros_pkgs" "robot_state_publisher" "tf2" "tf2_ros" "ros2_control")
set(project2_pkg_BUILDTOOL_DEPENDS "ament_cmake")
set(project2_pkg_BUILD_EXPORT_DEPENDS "rclpy" "rclcpp" "gazebo_ros_pkgs" "robot_state_publisher" "tf2" "tf2_ros" "ros2_control")
set(project2_pkg_BUILDTOOL_EXPORT_DEPENDS )
set(project2_pkg_EXEC_DEPENDS "rclpy" "rclcpp" "gazebo_ros_pkgs" "robot_state_publisher" "tf2" "tf2_ros" "ros2_control")
set(project2_pkg_TEST_DEPENDS "ament_lint_auto" "ament_lint_common" "ament_cmake_gtest")
set(project2_pkg_GROUP_DEPENDS )
set(project2_pkg_MEMBER_OF_GROUPS )
set(project2_pkg_DEPRECATED "")
set(project2_pkg_EXPORT_TAGS)
list(APPEND project2_pkg_EXPORT_TAGS "<build_type>ament_cmake</build_type>")
list(APPEND project2_pkg_EXPORT_TAGS "<gazebo_ros gazebo_model_path=\"..\"/>")
