cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++11)
project(judge_system_msgs)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  message_generation
)

add_message_files(
        FILES
        Alive_model.msg
        Alive_models.msg
        Bullet_launch.msg
        Move_cmd.msg
        Attack_cmd.msg
)
add_service_files(
  FILES
  clear_bullets.srv
)
generate_messages(
        DEPENDENCIES
        geometry_msgs
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS geometry_msgs std_msgs message_runtime
)
