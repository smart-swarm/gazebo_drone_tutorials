#!/usr/bin/env python
# coding=utf-8
'''
@Author       : LI Jinjie
@Date         : 2020-03-11 10:14:42
@LastEditors  : LI Jinjie
@LastEditTime : 2020-03-12 22:03:31
@Units        : Meter
@Description  : This file is a ROS node to receive tag_detections from apriltags node.
@Dependencies : None
@NOTICE       : None
'''
import cv2
import cv_bridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
import tf
import tf2_ros
import rospy
import numpy as np


class tf_sub:
    def __init__(self, colVec):
        # the airplane's coordination in the camera frame. column vector.
        self.camBodyV = colVec
        # quaternion xyzw
        self.q_xyzw = np.zeros((4))
        # the camera frame origin's coordination in the world frame
        self.wCamOrgV = np.zeros((3, 1))
        # the rotation matrix of the camera frame in the world frame
        self.wCamR = np.zeros((3, 3))
        # the airplane's coordination in the world frame. column vector.
        self.wBodyV = np.zeros((3, 1))
        # pose
        self.pose = Pose()

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def update_para(self):
        '''
        Update parameters.
        '''
        # child_frame_id, parent_frame_id, time, timeout
        # 5x5_bundle frame = world frame
        self.trans = self.tfBuffer.lookup_transform(
            'camera_frame', '5x5_bundle', rospy.Time())
        self.wCamOrgV[0, 0] = self.trans.transform.translation.x
        self.wCamOrgV[1, 0] = self.trans.transform.translation.y
        self.wCamOrgV[2, 0] = self.trans.transform.translation.z
        # 四元数法和旋转矩阵的换算
        self.dicQ = self.trans.transform.rotation
        self.q_xyzw[:] = [self.dicQ.x, self.dicQ.y, self.dicQ.z, self.dicQ.w]
        self.wCamR = tf.transformations.quaternion_matrix(self.q_xyzw)[:3, :3]
        self.wBodyV = np.dot(self.wCamR, self.camBodyV) + \
            self.wCamOrgV  # matrix multiplication
        self.pose.position.x = self.trans.transform.translation.x
        self.pose.position.y = self.trans.transform.translation.y
        self.pose.position.z = self.trans.transform.translation.z
        self.pose.orientation = self.trans.transform.rotation


class img_sub_show:
    def __init__(self, topic_name):
        self.cvBridge = cv_bridge.CvBridge()  # 类的实例
        self.img = 0
        self.topicName = topic_name

    def update_para(self):
        self.Sub = rospy.Subscriber(
            self.topicName, Image, self.img_callback, queue_size=10)

    def img_callback(self, img_data):
        self.img = self.cvBridge.imgmsg_to_cv2(img_data, 'bgr8')


if __name__ == '__main__':

    try:
        rospy.init_node('sub_tags_detections_test', anonymous=True)

        img_topic_name = "/tag_detections_image"
        img_sub_show_object = img_sub_show(img_topic_name)

        rowVec = np.array([0, 0, -0.045])  # 机体位置在camera_frame中的坐标
        colVec = rowVec.reshape(-1, 1)
        tf_sub_object = tf_sub(colVec)
        pub = rospy.Publisher('/apriltags_pose', Pose, queue_size=10)

        # {rospy.Rate(x), while loop and rate.sleep()} can make the loop happens x times one second.
        rate = rospy.Rate(30.0)  # 30Hz
        while not rospy.is_shutdown():
            try:
                tf_sub_object.update_para()
                pub.publish(tf_sub_object.pose)

                # img_sub_show_object.update_para()
                # cv2.imshow('img', img_sub_show_object.img)
                # cv2.waitKey(10)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
