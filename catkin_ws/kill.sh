#!/bin/bash

list='roslaunch 
      rosmaster
      rosout
      rostopic
      gzclient
      gzserver
      robot_state_publisher
      message_to_tf/message_to_tf
      controller_manager/spawner
      topic_tools/relay
      hector_quad_simple_ctrl/hector_quad_simple_ctrl_twist_node
      hector_quad_simple_ctrl/hector_quad_simple_ctrl_pose_node
      judge_system
		spawn_model
		kinetic'

for i in $list
    do
        ps -ef | grep $i | awk '{print $2}' | xargs -i kill -9 {}
    done

ps -ef | grep -v sim_platform_pysdk | grep catkin_ws | awk '{print $2}' | xargs -i kill -9 {}

sh python_kill.sh
