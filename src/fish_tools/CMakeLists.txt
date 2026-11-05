cmake_minimum_required(VERSION 3.0.2)
project(fish_tools)

find_package(catkin REQUIRED COMPONENTS
  fish_interfaces
  rospy
  std_msgs
  std_srvs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS fish_interfaces rospy std_msgs std_srvs
)

catkin_install_python(PROGRAMS
  scripts/experiment_manager.py
  scripts/keyboard_console.py
  scripts/bag_audit.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
