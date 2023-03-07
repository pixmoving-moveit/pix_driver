cmake_minimum_required(VERSION 3.5)
project(pix_hooke_driver_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# GearReport.msg
# {name}.msg
set(msg_files
	"msg/A2vBrakeCtrl.msg"
	"msg/A2vVehicleCtrl.msg"
	"msg/V2aChassisWheelAngleFb.msg"
	"msg/V2aDriveStaFb.msg"
	"msg/V2aVehicleFltSta.msg"
	"msg/A2vDriveCtrl.msg"
	"msg/A2vWheelCtrl.msg"
	"msg/V2aChassisWheelRpmFb.msg"
	"msg/V2aPowerStaFb.msg"
	"msg/V2aVehicleStaFb.msg"
	"msg/A2vSteerCtrl.msg"
	"msg/V2aBrakeStaFb.msg"
	"msg/V2aChassisWheelTirePressFb.msg"
	"msg/V2aSteerStaFb.msg"
	"msg/V2aVehicleWorkStaFb.msg"
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