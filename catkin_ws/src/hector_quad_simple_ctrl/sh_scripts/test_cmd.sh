#!/bin/bash
rostopic pub -r 10 /uav1/cmd_vel geometry_msgs/Twist  '{linear:  {x: 0, y: 0.0, z: 0.3}, angular: {x: 0.0,y: 0.0,z: 0.0}}' &PID0=$!
wait
kill PID0
exit

