#!/usr/bin/env python
# coding=utf-8
'''
@Author       : LI Jinjie
@Date         : 2020-02-23 22:21:54
@LastEditors  : LI Jinjie
@LastEditTime : 2020-02-29 20:46:23
@Units        : m, 
@Description  : This is the control node for a multi-agent system. Logical order: subscribe, control algorithm, publish. 我决定变量名用驼峰命名，函数名用下划线。
@Dependencies : ROS project gazebo_drone_tutorials
@NOTICE       : None
'''
import rospy
from nav_msgs.msg import Odometry
import numpy as np
import scipy
import myGlobal


'''
subscribe: /uav1/ground_truth/state, /uav2/ground_truth/state,......
type:nav_msgs/Odometry
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance
geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z
  float64[36] covariance
'''


def callback(data):
    myGlobal.posVel_1 = data


def subscribe():
    rospy.init_node('control_center', anonymous=True)
    rospy.Subscriber('/uav1/ground_truth/state', Odometry, callback)


if __name__ == '__main__':
    try:
        subscribe()
    except rospy.ROSInterruptException:
        pass
