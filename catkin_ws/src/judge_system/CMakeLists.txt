cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++11)
project(judge_system)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  roscpp
  std_msgs
  judge_system_msgs
)

find_package(Boost REQUIRED COMPONENTS
        system
        thread
        )
find_package(OpenCV REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS gazebo_msgs geometry_msgs roscpp std_msgs judge_system_msgs
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
#   ${GLOG_INCLUDE_DIRS}
)
# set(GLOG_LIB glog)
# target_link_libraries(${PROJECT_NAME} PUBLIC ${GLOG_LIB})

add_executable(${PROJECT_NAME}_node
        src/judge_interface.cpp
        src/authorized_check.cc
        src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
# target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${GLOG_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS} ${Boost_LIBRARIES})

add_executable(${PROJECT_NAME}_gui_node
        src/main_gui.cpp
        src/judge_gui.cpp)
# target_link_libraries(${PROJECT_NAME}_gui_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${GLOG_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_gui_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(${PROJECT_NAME}_gui_node ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_gui_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark all other required files for installation

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


