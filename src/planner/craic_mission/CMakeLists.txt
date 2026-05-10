cmake_minimum_required(VERSION 3.0.2)
project(craic_mission)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  quadrotor_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/mission_fsm.py
  scripts/mission_fsm_minimal.py
  scripts/drop_controller_mock.py
  scripts/fake_k230_publisher.py
  scripts/fake_odom.py
  scripts/fake_planner_inputs.py
  scripts/k230_bridge.py
  scripts/k230_serial_node.py
  scripts/safety_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
