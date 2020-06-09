cmake_minimum_required(VERSION 3.0.2)
project(pilsbot_control)

find_package(catkin REQUIRED)

catkin_package()

# catkin_install_python(PROGRAMS

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}

install(FILES config/diff_drive_controller.yaml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
install(FILES launch/pilsbot_control.launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
