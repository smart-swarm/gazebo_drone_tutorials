

###
 # @Author       : GUO ZhengLong, LI Jinjie
 # @Date         : 2020-03-06 15:26:32
 # @LastEditors  : LI Jinjie
 # @LastEditTime : 2020-03-19 12:14:01
 # @Units        : None
 # @Description  : file content
 # @Dependencies : None
 # @NOTICE       : None
 ###

#!/bin/bash

cd ./catkin_ws/
catkin build
source $(pwd)/devel/setup.bash
echo "loading gazebo world..."

# add gazebo model path
export CATKIN_PATH=$(pwd)
cd ./src/innok_heros_gazebo/worlds/Apriltags_world/models
source add_path.sh
cd $CATKIN_PATH

roslaunch innok_heros_gazebo load_world_apriltag_map.launch &
sleep 10
echo "loading uav and car..."
roslaunch hector_quadrotor_gazebo spawn_quadrotor_with_downward_cam.launch &
sleep 5

sleep 30
sh scripts/uav_arm.sh 1 /dev/null 2>&1 &
echo "all uav are ready to takeoff..."
sleep 10

# launch apriltags detection node
roslaunch apriltag_ros continuous_detection.launch &
echo "apriltag_ros node is running..."

echo "simulation platform ready..."
sleep 1
echo "simulation platform ready..."
sleep 1
echo "simulation platform ready..."
sleep 1
echo "simulation platform ready..."
sleep 1
echo "simulation platform ready..."
sleep 1
echo "simulation platform ready..."
sleep 1
echo "simulation platform ready..."
sleep 1
echo "simulation platform ready..."
sleep 1
echo "simulation platform ready..."
sleep 1
echo "simulation platform ready..."
wait

exit


