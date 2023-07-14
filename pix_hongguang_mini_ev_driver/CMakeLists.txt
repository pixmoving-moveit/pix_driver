cmake_minimum_required(VERSION 3.0.2)
project(pix_hongguang_mini_ev_driver)
set(CMAKE_BUILD_TYPE "Debug")
find_package(catkin REQUIRED COMPONENTS
  can_msgs
  roscpp
  std_msgs
  pix_hongguang_mini_ev_driver_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES pix_hongguang_mini_ev_driver
 CATKIN_DEPENDS can_msgs roscpp std_msgs pix_hongguang_mini_ev_driver_msgs
# DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/pix_driver.cpp
# )

# src/steering_report_502.cc
# src/{name}.cc
add_executable(${PROJECT_NAME}_report_node 
src/steeringreport_18f.cc
src/vhiclemodereport_200.cc
src/throttlereport_201.cc
src/gearreport_204.cc
src/brakereport_289.cc


src/report_node.cc
src/Byte.cc
)

# src/steering_command_102.cc
# src/{name}.cc
add_executable(${PROJECT_NAME}_command_node
src/vhiclemodecommand_100.cc
src/throttlecommand_101.cc
src/gearcommand_104.cc
src/steeringcomand_314.cc
src/brakecommand_364.cc


src/command_node.cc
src/Byte.cc
)

catkin_install_python(PROGRAMS src/control_converter.py src/report_converter_imu.py src/control_converter_test.py
src/out_auto_control.py src/control_converter_ok.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_dependencies(${PROJECT_NAME}_report_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_command_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_report_node
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_command_node
  ${catkin_LIBRARIES}
)


install(TARGETS ${PROJECT_NAME}_report_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_command_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
