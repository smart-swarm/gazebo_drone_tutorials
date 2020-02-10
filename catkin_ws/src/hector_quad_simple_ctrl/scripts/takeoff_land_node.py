#!/usr/bin/env python
import rospy
import sys
import time
import math
from std_msgs.msg import Header
from geometry_msgs.msg import Twist

cmd_vel_ctrl = Twist()
def takeoff_finished():
      print "have takeoff!"
def land_finished():
      print "land takeoff!"
def takeoff_land(drone, cmd, height):
    rospy.init_node('takeoff_land_node', anonymous=True)
    pub_topic_name = drone+"/cmd_vel"
    pub_cmd_vel = rospy.Publisher(pub_topic_name, Twist, queue_size=100)
    i = 0
    rate = rospy.Rate(25)
    t_delay = int(height*25)
    while not rospy.is_shutdown():
        if(cmd == "takeoff"): 
            if(i<t_delay):
                cmd_vel_ctrl.linear.z = 1
            # else:
            #     rospy.on_shutdown(takeoff_finished)
        elif(cmd == "land"):
            if(i<t_delay*2):
                cmd_vel_ctrl.linear.z = -1
            else:
                rospy.on_shutdown(land_finished)
        else:
            cmd_vel_ctrl.linear.z = 0.0
        i=i+1
        pub_cmd_vel.publish(cmd_vel_ctrl)
        cmd_vel_ctrl.linear.z = 0.0
        if(i>(t_delay+50)):  
            rospy.on_shutdown(takeoff_finished)
            time.sleep(3)
            sys.exit()
        rate.sleep()

if __name__ == '__main__':
    
    drone = str(sys.argv[1])
    cmd = str(sys.argv[2])
    height = float(sys.argv[3])
    # drone=input()
    # cmd=input()
    takeoff_land(drone, cmd, height)
