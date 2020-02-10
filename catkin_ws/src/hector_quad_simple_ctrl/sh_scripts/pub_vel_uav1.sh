#!/bin/bash
rostopic pub -r 10 /uav1/velocity_cmd geometry_msgs/Twist  '{linear:  {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.1}}'  & PID0=$!
wait
kill PID0
exit
