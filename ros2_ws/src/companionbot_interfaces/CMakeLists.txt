cmake_minimum_required(VERSION 3.16)
project(companionbot_interfaces)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SourceStamp.msg" "msg/ColorFrame.msg" "msg/MasterStatus.msg"
  "msg/RelativeObservation.msg" "msg/RobotSnapshot.msg" "msg/BehaviorFeedback.msg"
  "msg/ControlTelemetry.msg" "msg/PiMcuPacket.msg" "msg/McuFrame.msg" "msg/BalanceState.msg" "srv/UserInput.srv"
  "srv/NavigationControl.srv"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs)
ament_export_dependencies(rosidl_default_runtime)
# rosidl owns its generated Python package. Install the differently named,
# Binary codecs, asset paths and ROS adapter helpers with standard CMake, avoiding a
# second ament_python_install_package/rosidl target in the same project.
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(DIRECTORY companionbot_contracts DESTINATION "${PYTHON_INSTALL_DIR}"
  PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(source_stamps test/test_source_stamps.py)
endif()
ament_package()
