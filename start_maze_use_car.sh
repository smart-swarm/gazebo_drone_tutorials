#!/bin/bash

cd ./catkin_ws/
catkin_make
source $(pwd)/devel/setup.bash
echo "loading gazebo world..."
roslaunch innok_heros_gazebo load_hard_world.launch &
sleep 5
roslaunch sc_gazebo add_car_to_world.launch &
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


