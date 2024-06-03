cmake_minimum_required(VERSION 3.8)
project(nortek_nucleus_dvl)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  roscpp
  roslib
  tf
  tf_conversions
  pcl_ros
  pcl_conversions
  image_transport
)


add_message_files(
  FILES
  AHRS.msg
  Altimeter.msg
  BottomTrack.msg
  CurrentProfile.msg
  FieldCalibration.msg
  IMU.msg
  INS.msg
  Magnetometer.msg
)

add_service_files(
  FILES
  ConnectSerial.srv
  ConnectTcp.srv
  Disconnect.srv
  Start.srv
  StartFieldCalibration.srv
  Stop.srv
  Command.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


