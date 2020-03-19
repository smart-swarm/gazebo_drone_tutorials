#!/usr/bin/env python
# coding=utf-8
'''
@Author       : LI Jinjie
@Date         : 2020-03-11 10:14:42
@LastEditors  : LI Jinjie
@LastEditTime : 2020-03-19 20:01:36
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
import tf
import tf2_ros
import rospy
import numpy as np

from tools import tags_info_sub


class imu_info_sub:
    def __init__(self, imu_topic="/uav1/raw_imu"):
        self.imu_topic = imu_topic
        self.imu_info = Imu()
        self.wBodyP = np.zeros([3, 1])
        self.wBodyR = np.zeros([3, 3])

    def update_para(self):
        self.imu_sub = rospy.Subscriber(
            self.imu_topic, self.Imu, self.imu_info_processing, queue_size=100)

    def imu_info_processing(self, imu_info):
        self.imu_info = imu_info


if __name__ == '__main__':

    try:
        print "Running......"
        rospy.init_node('integrated_navigation_node', anonymous=True)

        cBodyOrgV = np.array([[0], [0], [-0.045]])
        tagsInfoSub_Obj = tags_info_sub(cBodyOrgV, sub_img_flag=False)
        pub = rospy.Publisher('/apriltags_pose', Pose, queue_size=10)

        # {rospy.Rate(x), while loop and rate.sleep()} can make the loop happens x times one second.
        rate = rospy.Rate(30.0)  # 30Hz
        while not rospy.is_shutdown():
            try:
                tagsInfoSub_Obj.update_para()

                pub.publish(tagsInfoSub_Obj.pose)

                # cv2.imshow('img', tagsInfoSub_Obj.img)
                # cv2.waitKey(10)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
