#!/usr/bin/env python
# coding=utf-8
'''
@Author       : LI Jinjie
@Date         : 2020-03-11 10:14:42
@LastEditors  : LI Jinjie
@LastEditTime : 2020-03-12 16:19:10
@Units        : Meter
@Description  : This file is a ROS node to receive tag_detections from apriltags node.
@Dependencies : None
@NOTICE       : None
'''
import numpy as np
import rospy
import tf2_ros

from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

import cv_bridge
import cv2


class tf_sub:
    def __init__(self, colVec):
        # the airplane's coordination in the camera frame. column vector.
        self.camBody = colVec
        # the camera frame origin's coordination in the world frame
        self.wrdCamOrg = np.zeros((3, 1))
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        '''
        Here, the real work is done, we query the listener for a specific transformation. Let's take a look at the arguments:

        1. We want the transform from this frame ...
        2. ... to this frame.
        3. The time at which we want to transform. Providing rospy.Time(0) will just get us the latest available transform. 
        '''
        # {rospy.Rate(x), while loop and rate.sleep()} can make the loop happens x times one second.
        self.rate = rospy.Rate(30.0)  # 10Hz
        while not rospy.is_shutdown():
            try:
                # child_frame_id, parent_frame_id, time, timeout
                # 5x5_bundle frame = world frame
                self.trans = self.tfBuffer.lookup_transform(
                    'camera_frame', '5x5_bundle', rospy.Time())
                self.wrdCamOrg[0, 0] = self.trans.transform.translation.x
                self.wrdCamOrg[1, 0] = self.trans.transform.translation.y
                self.wrdCamOrg[2, 0] = self.trans.transform.translation.z
                # 四元数法和旋转矩阵的换算
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.rate.sleep()
                continue


class img_sub_show:
    def __init__(self, topic_name):
        self.cvBridge = cv_bridge.CvBridge()  # 类的实例
        self.Sub = rospy.Subscriber(
            topic_name, Image, self.img_callback, queue_size=10)
        # self.rate = rospy.Rate(30)  # 30Hz
        self.img = 0

    def img_callback(self, img_data):
        self.img = self.cvBridge.imgmsg_to_cv2(img_data, 'bgr8')
        cv2.imshow('img', self.img)
        cv2.waitKey(10)


if __name__ == '__main__':
    try:
        rospy.init_node('sub_tags_detections_test', anonymous=True)

        img_topic_name = "/tag_detections_image"
        # img_sub_show_object = img_sub_show(img_topic_name)
        rowVec = np.array([0, 0, -0.045])  # 机体位置在camera_frame中的坐标
        colVec = rowVec.reshape(-1, 1)
        tf_sub_object = tf_sub(colVec)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
