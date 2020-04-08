#!/usr/bin/env python
# coding=utf-8
'''
@Author       : LI Jinjie
@Date         : 2020-03-13 09:48:44
@LastEditors  : LI Jinjie
@LastEditTime : 2020-03-27 10:01:15
@Units        : None
@Description  : file content
@Dependencies : None
@NOTICE       : 这里的Twist类型传递的的是位置信息！！！！！！
'''
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
# from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math
import time


def pub_vel_des_test():
    rospy.init_node('pub_vel_test')
    pub_vel_des = rospy.Publisher('/uav1/pose_cmd', Twist, queue_size=1)
    # pub_cmd = rospy.Publisher('/cmd_input', Twist, queue_size=1)
    rate = rospy.Rate(5)
    posNow = 0
    sign = 1
    i = 0
    start = time.time()
    while not rospy.is_shutdown():
        print(i)
        cmd_input = Twist()
        posNow = posNow + sign * 0.2
        if abs(posNow) >= 2.4:
            sign = -sign
        cmd_input.linear.x = 0
        cmd_input.linear.y = posNow
        cmd_input.linear.z = 1.5
        cmd_input.angular.x = 0.0
        cmd_input.angular.y = 0.0
        cmd_input.angular.z = 0.0
        i = i+1
        pub_vel_des.publish(cmd_input)
        # pub_cmd.publish(cmd_input)
        rate.sleep()


if __name__ == '__main__':
    try:
        pub_vel_des_test()
    except rospy.ROSInterruptException:
        pass
