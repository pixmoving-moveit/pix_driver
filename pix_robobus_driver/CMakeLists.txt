cmake_minimum_required(VERSION 3.5)
project(pix_robobus_driver)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

ament_auto_add_executable(${PROJECT_NAME}_control_command_node
  src/throttle_command.cc
  src/brake_command.cc
  src/steering_command.cc
  src/gear_command.cc
  src/park_command.cc
  src/vehicle_mode_command.cc

  src/Byte.cc
  src/control_command.cpp
  src/control_command_node.cpp

  src/auto_ctrl_msg.cc
  src/auto_remote_ctrl_msg.cc
)

ament_auto_add_executable(${PROJECT_NAME}_report_parser_node
  src/chassis_sta_to_mobileye.cc
  src/auto_ctrl_msg.cc
  src/auto_remote_ctrl_msg.cc
  src/throttle_report.cc
  src/brake_report.cc
  src/steering_report.cc
  src/gear_report.cc
  src/park_report.cc
  src/vcu_report.cc
  src/wheel_speed_report.cc
  src/bms_report.cc
  src/vehicle_door_report.cc
  src/vehicle_mileage_fb.cc
  src/vcu_pad_transfer.cc

  src/Byte.cc
  src/report_parser.cpp
  src/report_parser_node.cpp
)

ament_auto_add_executable(${PROJECT_NAME}_control_converter_node
  src/control_converter.cpp
  src/control_converter_node.cpp
)

ament_auto_add_executable(${PROJECT_NAME}_report_converter_node
  src/report_converter.cpp
  src/report_converter_node.cpp
)

# install
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  # config
)