cmake_minimum_required(VERSION 2.8.3)
project(innok_heros_gazebo)


find_package(catkin REQUIRED COMPONENTS roslaunch
)
catkin_package()

#roslaunch_add_file_check(launch)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

