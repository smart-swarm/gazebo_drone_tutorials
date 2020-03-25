#!/usr/bin/env python
# coding=utf-8
'''
@Author       : LI Jinjie
@Date         : 2020-03-19 16:04:46
@LastEditors  : LI Jinjie
@LastEditTime : 2020-03-25 15:21:44
@Units        : None
@Description  : This file defines the classes which are useful for integrated_navigation_node.py
@Dependencies : None
@NOTICE       : None
'''
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf
import tf2_ros
import rospy
import numpy as np
import cv2
import cv_bridge
from tf.transformations import *


class tags_info_sub:
    '''
    The target of this class is subscribing the apriltag_ros topic, processing the data and getting the pose from map frame -> base_link frame. The type of final result is PoseWithCovarianceStamped()
    '''
    MToBpose = PoseWithCovarianceStamped()

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
        # # pose--final result
        # self.pose = Pose()

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
            'camera_frame', 'map_frame', rospy.Time())

        # map frame -> camera frame  ===>  map frame -> base_link frame
        # 应该有用四元数直接变换的办法
        self.wCamOrgP[0, 0] = self.trans.transform.translation.x
        self.wCamOrgP[1, 0] = self.trans.transform.translation.y
        self.wCamOrgP[2, 0] = self.trans.transform.translation.z
        self.dicQ = self.trans.transform.rotation
        self.q_xyzw[:] = [self.dicQ.x, self.dicQ.y, self.dicQ.z, self.dicQ.w]
        self.wCamR = tf.transformations.quaternion_matrix(self.q_xyzw)[:3, :3]
        # self.wBodyP = np.dot(self.wCamR, self.camBodyV) + \
        #     self.wCamOrgP  # matrix multiplication

        # the object is located in the y=1. We use the format [x,y,z,w] and w is allways 0 for vectors
        r = self.trans.transform.rotation
        rot = [r.x, r.y, r.z, r.w]
        t = self.trans.transform.translation
        vec = [self.camBodyV[0], self.camBodyV[1], self.camBodyV[2], 0]
        # 四元数的姿态变换实际上就是把姿态矩阵变成四元数，矩阵乘法变成res=q*v*q'
        # cam坐标系在map坐标系中的姿态 × base原点在cam坐标系的中的位置 + cam坐标系在map坐标系中的位置

        # now we apply the mathematical operation res=q*v*q'. We use this function for multiplication but internally it is just a complex multiplication operation.
        result = quaternion_multiply(quaternion_multiply(
            rot, vec), quaternion_conjugate(rot)) + [[t.x], [t.y], [t.z], [0]]
        print 'Quaternion method', result[0:3]

        # update the position
        self.MToBpose.pose.pose.position.x = self.wBodyP[0]
        self.MToBpose.pose.pose.position.y = self.wBodyP[1]
        self.MToBpose.pose.pose.position.z = self.wBodyP[2]

        # update the orientation
        self.MToBpose.pose.pose.orientation = self.trans.transform.rotation

        # ========================

        # self.pose.position.x = self.trans.transform.translation.x
        # self.pose.position.y = self.trans.transform.translation.y
        # self.pose.position.z = self.trans.transform.translation.z
        # self.pose.orientation = self.trans.transform.rotation

    def img_callback(self, img_data):
        self.img = self.cvBridge.imgmsg_to_cv2(img_data, 'bgr8')
