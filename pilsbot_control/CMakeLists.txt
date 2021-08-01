cmake_minimum_required(VERSION 3.5.0)
project(pilsbot_control)

find_package(ament_cmake REQUIRED)

ament_package()

install(FILES config/diff_drive_controller.yaml
        DESTINATION share/${PROJECT_NAME}/config
)
install(FILES launch/pilsbot_control.launch
        DESTINATION share/${PROJECT_NAME}/launch
)
