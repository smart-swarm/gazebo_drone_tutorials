#!/bin/bash
#echo -n  "please enter the number of uav =:\n"
#read  num
#echo -n  "please enter the name of team =: A or B\n"
#read  team

for i in $(seq 1 $1);
do
    echo " uav$i armed.";
    rosservice call /uav$i/enable_motors true 
done
exit



