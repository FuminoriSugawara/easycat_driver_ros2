cmake_minimum_required(VERSION 3.8)
project(ethercat_easycat)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ethercat_driver REQUIRED)
find_package(ethercat_interface REQUIRED)
find_package(ethercat_generic_cia402_drive REQUIRED)
find_package(custom_position_controller REQUIRED)
find_package(custom_velocity_controller REQUIRED)
find_package(custom_effort_controller REQUIRED)
find_package(dynamic_controller REQUIRED)

install(
  DIRECTORY config description launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
