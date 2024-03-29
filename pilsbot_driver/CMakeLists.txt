cmake_minimum_required(VERSION 3.5.0)
project(pilsbot_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra) # -Wpedantic) #lol designated inicializers are better yo
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pilsbot_driver_msgs REQUIRED)

include_directories(
  include
  src/pid-controller
  src/hoverboard/src
  src/hoverboard/src/hbprotocol
)

set (HB_SRC
  src/hoverboard/src/protocolFunctions.c
  src/hoverboard/src/hbprotocol/ascii_protocol.c
  src/hoverboard/src/hbprotocol/protocol.c
  src/hoverboard/src/hbprotocol/cobsr.c
  src/hoverboard/src/hbprotocol/machine_protocol.c
  src/hoverboard/src/HoverboardAPI.cpp
)

# hoverboard_driver
add_library(pilsbot_driver
  SHARED
  ${HB_SRC}
  src/pid-controller/pid.cpp
  src/pilsbot_driver.cpp
)

ament_target_dependencies(pilsbot_driver
                          controller_manager #diagnostic_msgs
                          hardware_interface
                          rclcpp)

# testing hoverboard API
add_executable(hoverboard-api-testing
  ${HB_SRC}
  src/pid-controller/pid.cpp
  etc/hoverboard-api-testing.cpp
)
ament_target_dependencies(hoverboard-api-testing)

# sensors
add_executable(head_mcu_node
  src/head_mcu_node.cpp
)
ament_target_dependencies(head_mcu_node
  rclcpp
  pilsbot_driver_msgs) 


pluginlib_export_plugin_description_file(hardware_interface pilsbot_driver.xml)

ament_export_dependencies(controller_manager
                          hardware_interface
                          rclcpp)
                         
ament_export_include_directories(include)
ament_package()

install(TARGETS 
        pilsbot_driver
        head_mcu_node
        DESTINATION lib/${PROJECT_NAME}	#fixme: before it was directly lib, without projext name
)
install(DIRECTORY include/
        DESTINATION include
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)
