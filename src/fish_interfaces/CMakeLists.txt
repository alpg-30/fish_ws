cmake_minimum_required(VERSION 3.0.2)
project(fish_interfaces)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  FILES
  FlappingCommand.msg
  MotorTelemetry.msg
  GaitState.msg
  ExperimentState.msg
)

add_service_files(
  FILES
  SetGait.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
