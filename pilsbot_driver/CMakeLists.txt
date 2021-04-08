cmake_minimum_required(VERSION 3.5.0)
project(pilsbot_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(diagnostic_msgs REQUIRED)
# find_package(dynamic_reconfigure REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  include
  src/hoverboard/src
  src/hoverboard/src/hbprotocol
)

add_library(${PROJECT_NAME}
  SHARED
  src/pilsbot_driver.cpp
  src/hoverboard/src/HoverboardAPI.cpp
  src/hoverboard/src/protocolFunctions.c
  src/hoverboard/src/hbprotocol/ascii_protocol.c
  src/hoverboard/src/hbprotocol/protocol.c
  src/hoverboard/src/hbprotocol/machine_protocol.c)

pluginlib_export_plugin_description_file(hardware_interface pilsbot_driver.xml)

ament_target_dependencies(${PROJECT_NAME}
                          controller_manager
                          diagnostic_msgs
                          hardware_interface
                          rclcpp)

install(TARGETS ${PROJECT_NAME}
        DESTINATION lib
)
install(DIRECTORY include/
        DESTINATION include
)
install(FILES launch/pilsbot.launch launch/teleop.launch
        DESTINATION share/${PROJECT_NAME}}/launch
)
install(FILES config/ps4_pad.yaml DESTINATION share/${PROJECT_NAME}/config)

ament_export_dependencies(controller_manager
                         controller_manager
                         hardware_interface
                         #realtime_tools
                         rclcpp)
ament_export_include_directories(include)
ament_package()
