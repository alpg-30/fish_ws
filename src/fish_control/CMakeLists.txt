cmake_minimum_required(VERSION 3.0.2)
project(fish_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  fish_interfaces
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  diagnostic_msgs
)
find_package(Threads REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS fish_interfaces roscpp std_msgs std_srvs sensor_msgs diagnostic_msgs
)

set(FISH_DYNAMIXEL_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../DynamixelSDK/c++"
    CACHE PATH "ROBOTIS DynamixelSDK C++ root directory")

add_library(fish_control_logic
  src/motor_bus_mock.cpp
  src/motor_bus_dynamixel.cpp
  src/fluent_waveform.cpp
  src/config.cpp
  src/safety.cpp
)
target_include_directories(fish_control_logic PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

if(EXISTS "${FISH_DYNAMIXEL_SDK_DIR}/include/dynamixel_sdk/dynamixel_sdk.h")
  add_library(fish_dynamixel_sdk STATIC
    ${FISH_DYNAMIXEL_SDK_DIR}/src/dynamixel_sdk/group_bulk_read.cpp
    ${FISH_DYNAMIXEL_SDK_DIR}/src/dynamixel_sdk/group_bulk_write.cpp
    ${FISH_DYNAMIXEL_SDK_DIR}/src/dynamixel_sdk/group_sync_read.cpp
    ${FISH_DYNAMIXEL_SDK_DIR}/src/dynamixel_sdk/group_sync_write.cpp
    ${FISH_DYNAMIXEL_SDK_DIR}/src/dynamixel_sdk/group_fast_bulk_read.cpp
    ${FISH_DYNAMIXEL_SDK_DIR}/src/dynamixel_sdk/group_fast_sync_read.cpp
    ${FISH_DYNAMIXEL_SDK_DIR}/src/dynamixel_sdk/group_handler.cpp
    ${FISH_DYNAMIXEL_SDK_DIR}/src/dynamixel_sdk/packet_handler.cpp
    ${FISH_DYNAMIXEL_SDK_DIR}/src/dynamixel_sdk/port_handler.cpp
    ${FISH_DYNAMIXEL_SDK_DIR}/src/dynamixel_sdk/port_handler_linux.cpp
    ${FISH_DYNAMIXEL_SDK_DIR}/src/dynamixel_sdk/protocol1_packet_handler.cpp
    ${FISH_DYNAMIXEL_SDK_DIR}/src/dynamixel_sdk/protocol2_packet_handler.cpp
  )
  target_include_directories(fish_dynamixel_sdk PUBLIC
    "${FISH_DYNAMIXEL_SDK_DIR}/include"
    "${FISH_DYNAMIXEL_SDK_DIR}/include/dynamixel_sdk"
  )
  target_compile_definitions(fish_dynamixel_sdk PRIVATE LINUX _GNU_SOURCE)
  set_target_properties(fish_dynamixel_sdk PROPERTIES POSITION_INDEPENDENT_CODE ON)
  target_link_libraries(fish_dynamixel_sdk PUBLIC Threads::Threads)
  target_compile_definitions(fish_control_logic PRIVATE FISH_HAVE_DYNAMIXEL_SDK=1)
  target_link_libraries(fish_control_logic PUBLIC fish_dynamixel_sdk)
  message(STATUS "DynamixelSDK: ${FISH_DYNAMIXEL_SDK_DIR}")
else()
  message(WARNING
    "DynamixelSDK not found at ${FISH_DYNAMIXEL_SDK_DIR}; "
    "the dynamixel backend will fail at runtime instead of falling back to mock")
endif()

add_executable(controller_node src/controller_node.cpp)
target_link_libraries(controller_node fish_control_logic ${catkin_LIBRARIES})
add_dependencies(controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS controller_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  enable_testing()
  foreach(test_name IN ITEMS test_fluent_phase test_geometry test_config test_safety)
    add_executable(${test_name} test/${test_name}.cpp)
    target_include_directories(${test_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(${test_name} fish_control_logic)
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()
endif()
