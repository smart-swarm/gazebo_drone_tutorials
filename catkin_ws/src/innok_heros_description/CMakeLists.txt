cmake_minimum_required(VERSION 2.8.3)
project(innok_heros_description)

find_package(catkin REQUIRED COMPONENTS
  robot_state_publisher
  roslaunch
  urdf
  xacro
)
catkin_package()
roslaunch_add_file_check(launch)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


install(
  DIRECTORY launch urdf meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
