cmake_minimum_required(VERSION 3.0.2)
project(pix_mini_ev_driver)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  can_msgs
  roscpp
  std_msgs
  pix_mini_ev_driver_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES pix_mini_ev_driver
 CATKIN_DEPENDS autoware_msgs can_msgs roscpp std_msgs pix_mini_ev_driver_msgs
# DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/pix_mini_ev_driver.cpp
# )

# src/steering_report_502.cc
# src/{name}.cc
add_executable(${PROJECT_NAME}_report_node 
src/eps_send2_18e.cc
src/eps_send_18f.cc
src/msg200_200.cc
src/msg201_201.cc
src/msg204_204.cc
src/receive_289.cc
src/eps_receive_314.cc
src/eps_receive2_315.cc
src/sendx_364.cc


src/report_node.cc
src/Byte.cc
)

# src/steering_command_102.cc
# src/{name}.cc
add_executable(${PROJECT_NAME}_command_node
src/msg100_100.cc
src/msg101_101.cc
src/msg104_104.cc


src/command_node.cc
src/Byte.cc
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

catkin_install_python(PROGRAMS src/report_converter.py src/control_converter.py src/report_converter_imu.py src/control_converter_speed.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(TARGETS ${PROJECT_NAME}_report_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_command_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)


