#!/bin/bash
rosnode kill /computer /player
sh ./scripts/uav_disarm.sh 3 3
rosservice call /reset_plat true
rosnode kill /judge_system_node /judge_system_gui_node
ps -ef | grep judge_system | awk '{print $2}' | xargs -i kill -9 {}
sh ./scripts/reset_to_initial_pose.sh
sh ./scripts/uav_arm.sh 3 3
roslaunch judge_system judge_system_car6_uav6.launch path:=$(pwd)
