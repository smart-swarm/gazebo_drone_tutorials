#!/bin/bash

cd ./catkin_ws/
catkin_make
source $(pwd)/devel/setup.bash
echo "loading gazebo world..."
roslaunch innok_heros_gazebo load_hard_world.launch &
sleep 10
echo "loading uav and car..."
roslaunch hector_quadrotor_gazebo spawn_one_quadrotors_for_maze_hard.launch &
sleep 5

sleep 30
sh scripts/uav_arm.sh 1 /dev/null 2>&1 &
echo "all uav are ready to takeoff..."
sleep 5
roslaunch sc_gazebo add_car_to_world.launch &
sleep 5
python ../judge_uav_solve_maze.py
sleep 5

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


