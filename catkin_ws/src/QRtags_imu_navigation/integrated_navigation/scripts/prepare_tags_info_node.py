#!/usr/bin/env python
# coding=utf-8
'''
@Author       : LI Jinjie
@Date         : 2020-03-11 10:14:42
@LastEditors  : LI Jinjie
@LastEditTime : 2020-03-28 23:12:28
@Units        : Meter
@Description  : This file is a ROS node to receive tag_detections from apriltags node.
@Dependencies : tools.py
@NOTICE       : w: world frame, b: body_frame, c: camera_frame, Org: origin point, R: Rotation matrix, P: Position vector, V: Velocity vector. cBodyOrgV: the position vector that where the body frame's origin point is in the camera frame.
'''
import cv2
import cv_bridge

from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Vector3
import tf
import tf2_ros
import rospy
import numpy as np

from tools import tags_sub


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


if __name__ == '__main__':

    try:
        print "prepare_tags_info_node is running......"
        rospy.init_node('prepare_tags_info_node', anonymous=False)
        ns = rospy.get_namespace()
        topic_name = ns + 'tags_pose'

        # the coordination that base_link's origin in camera frame
        cBodyOrgV = np.array([[0], [0], [-0.045]])
        tagsSub_Obj = tags_sub(cBodyOrgV, sub_img_flag=False)
        tagsPublisher = rospy.Publisher(
            topic_name, PoseWithCovarianceStamped, queue_size=10)

        oldStamp = 0

        # {rospy.Rate(x), while loop and rate.sleep()} can make the loop happens x times one second.
        rate = rospy.Rate(30.0)  # 30Hz
        while not rospy.is_shutdown():
            try:

                tagsSub_Obj.update_para()
                # publish only when info changed
                if oldStamp != tagsSub_Obj.trans.header.stamp:
                    tagsPublisher.publish(tagsSub_Obj.MToBpose)
                oldStamp = tagsSub_Obj.trans.header.stamp

                # cv2.imshow('img', tagsSub_Obj.img)
                # cv2.waitKey(10)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
