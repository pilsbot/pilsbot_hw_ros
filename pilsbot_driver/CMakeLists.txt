cmake_minimum_required(VERSION 3.0.2)
project(pilsbot_driver)

find_package(catkin REQUIRED COMPONENTS controller_manager diagnostic_msgs dynamic_reconfigure hardware_interface realtime_tools roscpp std_msgs)

catkin_package(
    CATKIN_DEPENDS
    controller_manager
        diagnostic_msgs
        dynamic_reconfigure
        hardware_interface
        realtime_tools
        roscpp
        std_msgs
    INCLUDE_DIRS include
)

include_directories(
  include
  src/hoverboard/src
  src/hoverboard/src/hbprotocol
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node 
  src/pilsbot_driver_node.cpp
  src/pilsbot_driver.cpp
  src/hoverboard/src/HoverboardAPI.cpp
  src/hoverboard/src/protocolFunctions.c
  src/hoverboard/src/hbprotocol/ascii_protocol.c
  src/hoverboard/src/hbprotocol/protocol.c
  src/hoverboard/src/hbprotocol/machine_protocol.c)

# catkin_install_python(PROGRAMS

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES launch/pilsbot.launch launch/teleop.launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(FILES config/ps4_pad.yaml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
