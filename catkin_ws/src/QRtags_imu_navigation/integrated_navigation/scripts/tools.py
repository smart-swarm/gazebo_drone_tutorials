#!/usr/bin/env python
# coding=utf-8
'''
@Author       : LI Jinjie
@Date         : 2020-03-19 16:04:46
@LastEditors  : LI Jinjie
@LastEditTime : 2020-03-28 20:47:32
@Units        : ROS REP-103: https://www.ros.org/reps/rep-0103.html
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
from rosgraph_msgs.msg import Clock


class tags_sub:
    '''
    The target of this class is subscribing the apriltag_ros topic, processing the data and getting the pose from map frame -> base_link frame. The type of final result is PoseWithCovarianceStamped()
    '''
    MToBpose = PoseWithCovarianceStamped()

    def __init__(self, cBodyOrgV, sub_img_flag=False, img_topic="/uav1/tag_detections_image"):
        # to show images
        self.sub_img_flag = sub_img_flag
        if self.sub_img_flag == True:
            self.cvBridge = cv_bridge.CvBridge()
            self.img_topic = img_topic
            self.img = 0

        # INITIALIZE PoseWithCovarianceStamped()
        self.MToBpose.header.seq = 0
        # frame_id is NOT the world_frame parameter (typically map or odom)
        # nameSpace = rospy.get_namespace()
        self.MToBpose.header.frame_id = 'uav1/map'

        cov = np.eye(6)
        cov[0, 0] = np.square(0.04)  # (x error)^2
        cov[1, 1] = np.square(0.04)  # (y error)^2
        cov[2, 2] = np.square(0.08)  # (z error)^2
        self.MToBpose.pose.covariance = tuple(cov.ravel().tolist())

        # the airplane's coordination in the camera frame. column vector.
        self.camBodyV = cBodyOrgV

        self.tfBuffer = tf2_ros.Buffer()
        self.TFlistener = tf2_ros.TransformListener(self.tfBuffer)
        self.clockListener = rospy.Subscriber(
            '/clock', Clock, self.update_time)

    def update_time(self, data):
        self.MToBpose.header.stamp = data.clock
        # print 'time =', time.clock

    def update_para(self):
        '''
        Update parameters. Prepare data for robot localization.
        '''
        if self.sub_img_flag == True:  # if para exists
            self.img_sub = rospy.Subscriber(
                self.img_topic, Image, self.img_callback, queue_size=10)

        # 0. update the header. the header.seq has been deprecated.
        # self.MToBpose.header.stamp = rospy.Time.now()

        # 1. get "map -> camera" transform
        # child_frame_id, parent_frame_id, time, timeout
        self.trans = self.tfBuffer.lookup_transform(
            'camera_frame', 'map_frame', rospy.Time())

        # 2. "map frame -> camera frame"  ===>  "map frame -> base_link frame"
        # 四元数的姿态变换实际上就是把姿态矩阵变成四元数，矩阵乘法变成res=q*v*q'
        # cam坐标系在map坐标系中的姿态 × base原点在cam坐标系的中的位置 + cam坐标系在map坐标系中的位置

        # the object is located in the y=1. We use the format [x,y,z,w] and w is allways 0 for vectors
        vec = [self.camBodyV[0], self.camBodyV[1], self.camBodyV[2], 0]
        r = self.trans.transform.rotation
        rot = [r.x, r.y, r.z, r.w]
        t = self.trans.transform.translation

        # now we apply the mathematical operation res=q*v*q'. We use this function for multiplication but internally it is just a complex multiplication operation.
        result = quaternion_multiply(quaternion_multiply(
            rot, vec), quaternion_conjugate(rot)) + [[t.x], [t.y], [t.z], [0]]
        # print 'Quaternion method', result[0:3]

        # 3. update the position
        self.MToBpose.pose.pose.position.x = result[1]
        self.MToBpose.pose.pose.position.y = result[0]
        self.MToBpose.pose.pose.position.z = result[2]

        # 4. update the orientation
        self.MToBpose.pose.pose.orientation = self.trans.transform.rotation

        # 5. the covariance matrix is stable and has been define at the top

    def img_callback(self, img_data):
        self.img = self.cvBridge.imgmsg_to_cv2(img_data, 'bgr8')
