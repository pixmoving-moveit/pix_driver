cmake_minimum_required(VERSION 3.0.2)
project(pix_hongguang_mini_ev_driver_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  std_msgs
)

file(GLOB MSGS_PATH ./mgs/*.msg)

# GearReport.msg
# {name}.msg
add_message_files(
  FILES
  	steeringreport_18f.msg
	vhiclemodereport_200.msg
	throttlereport_201.msg
	gearreport_204.msg
	brakereport_289.msg
	vhiclemodecommand_100.msg
	throttlecommand_101.msg
	gearcommand_104.msg
	steeringcomand_314.msg
	brakecommand_364.msg

)

generate_messages(
  DEPENDENCIES
  std_msgs
  pix_hongguang_mini_ev_driver_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pix_driver_msg
#  CATKIN_DEPENDS message_generation message_runtime roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
