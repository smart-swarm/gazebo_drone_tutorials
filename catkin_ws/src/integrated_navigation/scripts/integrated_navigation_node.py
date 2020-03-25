#!/usr/bin/env python
# coding=utf-8
'''
@Author       : LI Jinjie
@Date         : 2020-03-11 10:14:42
@LastEditors  : LI Jinjie
@LastEditTime : 2020-03-25 14:55:52
@Units        : Meter
@Description  : This file is a ROS node to receive tag_detections from apriltags node.
@Dependencies : tools.py
@NOTICE       : w: world frame, b: body_frame, c: camera_frame, Org: origin point, R: Rotation matrix, P: Position vector, V: Velocity vector. cBodyOrgV: the position vector that where the body frame's origin point is in the camera frame.
'''
import cv2
import cv_bridge

from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
import tf
import tf2_ros
import rospy
import numpy as np

from tools import tags_info_sub


class imu_info_sub:
    def __init__(self, ti, imu_topic="/uav1/raw_imu"):
        self.imu_topic = imu_topic
        self.imu_info = Imu()
        # initialization. 需要进一步修正
        self.wBodyP = np.zeros([3, 1])
        self.wBodyV = np.zeros([3, 1])
        self.wBodyR = np.eye(3)
        self.ti = ti
        self.g = 9.80665

    def update_para(self):
        self.imu_sub = rospy.Subscriber(
            self.imu_topic, Imu, self.imu_info_processing, queue_size=100)

    def imu_info_processing(self, imu_info):
        self.imu_info = imu_info
        wBodyR_old = self.wBodyR

        # Attitude Update
        M = np.eye(3)
        M[0, 1] = - self.imu_info.angular_velocity.z * self.ti
        M[0, 2] = self.imu_info.angular_velocity.y * self.ti
        M[1, 0] = self.imu_info.angular_velocity.z * self.ti
        M[1, 2] = - self.imu_info.angular_velocity.x * self.ti
        M[2, 0] = - self.imu_info.angular_velocity.y * self.ti
        M[2, 1] = self.imu_info.angular_velocity.x * self.ti
        self.wBodyR = self.wBodyR * M

        # Specific-Force Frame Transformation
        bSpeForce = np.array([[self.imu_info.linear_acceleration.x], [
                             self.imu_info.linear_acceleration.y], [self.imu_info.linear_acceleration.z]])
        wSpeForce = np.dot((wBodyR_old + self.wBodyR) / 2, bSpeForce)

        # Velocity Update
        revision = np.array([[0], [0], [-self.g]])
        wAcc = wSpeForce + revision
        self.wBodyV = self.wBodyV + wAcc * self.ti

        # Position Update
        self.wBodyP = self.wBodyP + self.wBodyV * \
            self.ti - wAcc * np.square(self.ti) / 2


if __name__ == '__main__':

    try:
        print "Running......"
        rospy.init_node('integrated_navigation_node', anonymous=True)

        cBodyOrgV = np.array([[0], [0], [-0.045]])
        tagsInfoSub_Obj = tags_info_sub(cBodyOrgV, sub_img_flag=False)
        ti = 1.0/100.0
        imuInfoSub_Obj = imu_info_sub(ti)
        tagsPub = rospy.Publisher('/apriltags_pose', Pose, queue_size=10)
        imuPub = rospy.Publisher('/imu_position', Vector3, queue_size=10)
        point = Vector3()

        # {rospy.Rate(x), while loop and rate.sleep()} can make the loop happens x times one second.
        rate = rospy.Rate(100.0)  # 100Hz
        i = 0
        while not rospy.is_shutdown():
            imuInfoSub_Obj.update_para()
            if i == 40:
                try:
                    tagsInfoSub_Obj.update_para()

                    # 修正imu的
                    imuInfoSub_Obj.wBodyP = tagsInfoSub_Obj.wBodyP

                    # cv2.imshow('img', tagsInfoSub_Obj.img)
                    # cv2.waitKey(10)

                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    pass
                i = 0
            i = i + 1

            point.x = imuInfoSub_Obj.wBodyP[0]
            point.y = imuInfoSub_Obj.wBodyP[1]
            point.z = imuInfoSub_Obj.wBodyP[2]
            imuPub.publish(point)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
