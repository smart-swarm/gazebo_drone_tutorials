#!/bin/bash
num=$1
list='uavA1 
      uavA2
      uavA3
      carA1
      carA2
carA3
      uavB1 
      uavB2
      uavB3
      carB1
      carB2
carB3'
echo $num
j=2
for name in $list
do
	echo $name
    for i in $(seq 1 $num);
	do
		echo $i
		echo "${name}_bullet_${i}"
		rosrun gazebo_ros spawn_model -file ~/.gazebo/models/my_bullet_control/model.sdf -sdf -x $i -y $(($j+500)) -z 10 -model "${name}_bullet_${i}" &
	done
	j=$(($j+2))
done

