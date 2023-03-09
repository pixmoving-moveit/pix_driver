cmake_minimum_required(VERSION 3.5)
project(pix_hooke_driver)

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


# Declare a C++ executable
ament_auto_add_executable(${PROJECT_NAME}_control_command_node
  src/a2v_drivectrl_130.cc
  src/a2v_brakectrl_131.cc
  src/a2v_steerctrl_132.cc
  src/a2v_vehiclectrl_133.cc
  src/a2v_wheelctrl_135.cc
  src/Byte.cc
  src/control_command.cpp
  src/control_command_node.cpp
)

ament_auto_add_executable(${PROJECT_NAME}_report_parser_node
  src/v2a_brakestafb_531.cc
  src/v2a_chassiswheelanglefb_541.cc
  src/v2a_chassiswheelrpmfb_539.cc
  src/v2a_chassiswheeltirepressfb_540.cc
  src/v2a_drivestafb_530.cc
  src/v2a_powerstafb_535.cc
  src/v2a_steerstafb_532.cc
  src/v2a_vehiclefltsta_537.cc
  src/v2a_vehiclestafb_536.cc
  src/v2a_vehicleworkstafb_534.cc
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
