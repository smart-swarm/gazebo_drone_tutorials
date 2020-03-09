

###
 # @Author       : LI Jinjie
 # @Date         : 2020-03-06 15:26:32
 # @LastEditors  : LI Jinjie
 # @LastEditTime : 2020-03-07 11:55:27
 # @Units        : None
 # @Description  : file content
 # @Dependencies : None
 # @NOTICE       : None
 ###
#!/bin/bash

cd ./catkin_ws/
catkin_make
source $(pwd)/devel/setup.bash
echo "loading gazebo world..."
roslaunch innok_heros_gazebo load_world_Apriltag_5x5.launch &
sleep 10
echo "loading uav and car..."
roslaunch hector_quadrotor_gazebo spawn_quadrotor_with_downward_cam.launch &
sleep 5

sleep 30
sh scripts/uav_arm.sh 1 /dev/null 2>&1 &
echo "all uav are ready to takeoff..."
sleep 10

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


