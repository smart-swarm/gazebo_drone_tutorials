#!/usr/bin/env python
# coding=utf-8
'''
@Author       : LI Jinjie
@Date         : 2020-03-19 16:04:46
@LastEditors  : LI Jinjie
@LastEditTime : 2020-03-19 20:14:26
@Units        : None
@Description  : This file defines the classes which are useful for integrated_navigation_node.py
@Dependencies : None
@NOTICE       : None
'''
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
import tf
import tf2_ros
import rospy
import numpy as np
import cv2
import cv_bridge


class tags_info_sub:
    '''
    The target of this class is subscribing the apriltag_ros topic, processing the data and getting the posture.
    '''

    def __init__(self, cBodyOrgV, sub_img_flag=False, img_topic="/tag_detections_image"):
        self.sub_img_flag = sub_img_flag
        if self.sub_img_flag == True:
            self.cvBridge = cv_bridge.CvBridge()  # 类的实例
            self.img_topic = img_topic
            self.img = 0

        # the airplane's coordination in the camera frame. column vector.
        self.camBodyV = cBodyOrgV
        # quaternion xyzw
        self.q_xyzw = np.zeros((4))
        # the camera frame origin's coordination in the world frame
        self.wCamOrgP = np.zeros((3, 1))
        # the rotation matrix of the camera frame in the world frame
        self.wCamR = np.eye(3)
        # the airplane's coordination in the world frame. column vector.
        self.wBodyP = np.zeros((3, 1))
        # pose--final result
        self.pose = Pose()

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def update_para(self):
        '''
        Update parameters.
        '''
        if self.sub_img_flag == True:  # if para exists
            self.img_sub = rospy.Subscriber(
                self.img_topic, Image, self.img_callback, queue_size=10)

        # child_frame_id, parent_frame_id, time, timeout
        # 5x5_bundle frame = world frame
        self.trans = self.tfBuffer.lookup_transform(
            'camera_frame', '5x5_bundle', rospy.Time())
        self.wCamOrgP[0, 0] = self.trans.transform.translation.x
        self.wCamOrgP[1, 0] = self.trans.transform.translation.y
        self.wCamOrgP[2, 0] = self.trans.transform.translation.z
        # 四元数法和旋转矩阵的换算
        self.dicQ = self.trans.transform.rotation
        self.q_xyzw[:] = [self.dicQ.x, self.dicQ.y, self.dicQ.z, self.dicQ.w]
        self.wCamR = tf.transformations.quaternion_matrix(self.q_xyzw)[:3, :3]
        self.wBodyP = np.dot(self.wCamR, self.camBodyV) + \
            self.wCamOrgP  # matrix multiplication
        self.pose.position.x = self.trans.transform.translation.x
        self.pose.position.y = self.trans.transform.translation.y
        self.pose.position.z = self.trans.transform.translation.z
        self.pose.orientation = self.trans.transform.rotation

    def img_callback(self, img_data):
        self.img = self.cvBridge.imgmsg_to_cv2(img_data, 'bgr8')
