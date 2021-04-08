cmake_minimum_required(VERSION 3.5.0)
project(pilsbot_control)

find_package(ament_cmake REQUIRED)

ament_package()

# catkin_install_python(PROGRAMS

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}

install(FILES config/diff_drive_controller.yaml
        DESTINATION share/${PROJECT_NAME}/config
)
install(FILES launch/pilsbot_control.launch
        DESTINATION share/${PROJECT_NAME}/launch
)
