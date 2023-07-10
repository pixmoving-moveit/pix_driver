cmake_minimum_required(VERSION 3.5)
project(pix_sweeping_driver_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(msg_files
		"msg/Vcu2AcuChassisErrCode1.msg"
	"msg/Vcu2AcuChassisErrCode2.msg"
	"msg/Vcu2AcuChassisErrCode3.msg"
	"msg/ThrottleReport.msg"
	"msg/BrakeReport.msg"
	"msg/SteeringReport.msg"
	"msg/GearReport.msg"
	"msg/ParkReport.msg"
	"msg/VcuReport.msg"
	"msg/WheelSpeedReport.msg"
	"msg/BmsReport.msg"
	"msg/Vcu2AcuSweepStaFb.msg"
	"msg/Vcu2AcuSweepFanSta.msg"
	"msg/Vcu2AcuSweepSta.msg"
	"msg/ScuWorkTimeFb.msg"
	"msg/Vcu2AcuSweepWorkSta.msg"
	"msg/VehicleMileageFb.msg"
	"msg/ThrottleCommand.msg"
	"msg/BrakeCommand.msg"
	"msg/SteeringCommand.msg"
	"msg/GearCommand.msg"
	"msg/ParkCommand.msg"
	"msg/VehicleModeCommand.msg"
	"msg/AcuSweepCtrlCmd.msg"

)

set(msg_dependencies
	std_msgs
  builtin_interfaces)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES ${msg_dependencies}
  ADD_LINTER_TESTS)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()