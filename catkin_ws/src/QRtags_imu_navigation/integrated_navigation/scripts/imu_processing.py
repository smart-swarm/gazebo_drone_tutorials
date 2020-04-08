#!/usr/bin/env python
# coding=utf-8
'''
@Author       : LI Jinjie
@Date         : 2020-03-27 11:24:31
@LastEditors  : LI Jinjie
@LastEditTime : 2020-03-28 23:06:35
@Units        : None
@Description  : file content
@Dependencies : Used to test imu msg. Useless for normally use
@NOTICE       : None
'''
import rospy
from sensor_msgs.msg import Imu
import numpy as np

imu = Imu()


def imu_callback(data):
    global imu
    imu.header.frame_id = 'uav1/odom'
    imu = data
    cov = np.eye(3)
    cov[0, 0] = np.square(0.1)  # (x error)^2
    cov[1, 1] = np.square(0.1)  # (y error)^2
    cov[2, 2] = np.square(0.1)  # (z error)^2
    imu.linear_acceleration_covariance = tuple(cov.ravel().tolist())


def sub():
    sub = rospy.Subscriber('/uav1/raw_imu', Imu, imu_callback)


def pub():
    global imu
    rospy.init_node('imu_processing', anonymous=True)
    print 'imu_processing_node is running......'

    sub = rospy.Subscriber('/uav1/raw_imu', Imu, imu_callback)
    pub = rospy.Publisher('/imu_new', Imu, queue_size=10)

    rate = rospy.Rate(100)  # 100hz
    while not rospy.is_shutdown():
        pub.publish(imu)
        rate.sleep()


if __name__ == '__main__':
    try:
        pub()
    except rospy.ROSInterruptException:
        pass
