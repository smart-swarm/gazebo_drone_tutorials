cmake_minimum_required(VERSION 2.8)
project(sim_platform_pysdk)

find_package(cmake_modules REQUIRED)
find_package(catkin REQUIRED)
catkin_package()

# catkin_python_setup()


install(PROGRAMS src/platform_interface.py  
		 src/example.py  
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)