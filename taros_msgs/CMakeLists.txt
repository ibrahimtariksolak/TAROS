cmake_minimum_required(VERSION 3.8)
project(taros_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Track.msg"
  "msg/TrackArray.msg"
  "msg/DroneFlightPlan.msg"
  "msg/RadarReturn.msg"
  "msg/RadarReturnArray.msg"
  "msg/AcousticDetection.msg"
  "msg/AcousticDetectionArray.msg"
  "msg/EoirDetection.msg"
  "msg/EoirDetectionArray.msg"
  "msg/ThreatAssessment.msg"
  "msg/EngagementCommand.msg"
  "msg/WeaponTargetAssignment.msg"
  "msg/RFSignal.msg"
  "msg/OperatorLocation.msg"
  "msg/WeatherCondition.msg"
  "msg/SystemStatus.msg"
  "msg/EngagementRecord.msg"
  "srv/Engage.srv"
  "srv/AuthorizeEngagement.srv"
  "srv/SpawnDrone.srv"
  "srv/SetWeather.srv"
  "action/InterceptTarget.action"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs nav_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
