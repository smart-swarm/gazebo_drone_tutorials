#!/bin/bash
#echo -n  "please enter the number of uav =:\n"
#read  num
#echo -n  "please enter the name of team =: A or B\n"
#read  team

for i in $(seq 1 $1);
do
    echo " A:uav$i armed.";
    rosservice call /A/uav$i/enable_motors false 
done

for i in $(seq 1 $2);
do
    echo " B:uav$i armed.";
    rosservice call /B/uav$i/enable_motors false
done

exit



