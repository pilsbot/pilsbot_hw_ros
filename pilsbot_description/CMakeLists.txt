cmake_minimum_required(VERSION 3.0.2)
project(pilsbot_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro
)

catkin_package(
#  CATKIN_DEPENDS urdf xacro
)

install(FILES urdf/pilsbot.urdf.xacro DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)
