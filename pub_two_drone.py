#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
# from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math
import time


def pub_vel_des_test():
    rospy.init_node('pub_vel_test')
    pub_vel_des = rospy.Publisher('/pose_cmd', Twist, queue_size=1)
    # pub_cmd = rospy.Publisher('/cmd_input', Twist, queue_size=1)
    rate = rospy.Rate(10)
    i = 0
    start = time.time()
    while not rospy.is_shutdown():
        print(i)
        cmd_input = Twist()
        t = 6.28/20.0 * (time.time() - start)
        cmd_input.linear.x = 6 * math.cos(t)
        cmd_input.linear.y = 6 * math.sin(t)
        cmd_input.linear.z = 4
        cmd_input.angular.x = 0.0
        cmd_input.angular.y = 0.0
        cmd_input.angular.z = 0.0

        # msg_data = Vector3()
        #
        # msg_data.x  =  5 * math.cos(2*3.14/50*i)
        # msg_data.y  =  5 * math.sin(2*3.14/50*i)
        # msg_data.z  =  0.0
        i = i+1
        pub_vel_des.publish(cmd_input)
        # pub_cmd.publish(cmd_input)
        rate.sleep()

if __name__ == '__main__':
    try:
        pub_vel_des_test()
    except rospy.ROSInterruptException: pass