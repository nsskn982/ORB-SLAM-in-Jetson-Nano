cmake_minimum_required(VERSION 3.0.2)
project(jetbot_vslam)

## Find catkin and required ROS components
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    std_srvs
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf
)

## All ROS nodes are Python; install the executable scripts.
catkin_install_python(PROGRAMS
  scripts/occupancy_grid_node.py
  scripts/map_collision_node.py
  scripts/collision_node.py
  scripts/decision_node.py
  scripts/encoder_node.py
  scripts/motor_feedback_controller.py
  scripts/battery_monitor_node.py
  scripts/telemetry_logger_node.py
  scripts/map_persistence_node.py
  scripts/imx219_camera_test.py
  scripts/imx219_camera_calibrate.py
  scripts/qwiic_bus_scan.py
  scripts/qwiic_motor_driver_test.py
  scripts/qwiic_oled_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch and config files.
install(DIRECTORY launch config models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)
