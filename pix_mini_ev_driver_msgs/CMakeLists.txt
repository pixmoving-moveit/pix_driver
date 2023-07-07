cmake_minimum_required(VERSION 3.0.2)
project(pix_mini_ev_driver_msgs)

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
  	eps_send2_18e.msg
	eps_send_18f.msg
	msg200_200.msg
	msg201_201.msg
	msg204_204.msg
	receive_289.msg
	eps_receive_314.msg
	eps_receive2_315.msg
	sendx_364.msg
	msg100_100.msg
	msg101_101.msg
	msg104_104.msg

)

generate_messages(
  DEPENDENCIES
  std_msgs
  pix_mini_ev_driver_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pix_mini_ev_driver_msg
#  CATKIN_DEPENDS message_generation message_runtime roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
