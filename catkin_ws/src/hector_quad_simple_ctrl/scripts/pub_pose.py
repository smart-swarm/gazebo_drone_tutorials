#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import Header
from geometry_msgs.msg import Twist

pose_vel_ctrl = Twist()

def fly2goal(drone, X,Y,Z,Yaw):
    rospy.init_node('fly2goal', anonymous=True)
    pub_topic_name = drone+"/pose_cmd"
    pub_pose_cmd = rospy.Publisher(pub_topic_name, Twist, queue_size=100)
    i = 0
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        pose_vel_ctrl.linear.x = X;
        pose_vel_ctrl.linear.y = Y;
        pose_vel_ctrl.linear.z = Z;
        pose_vel_ctrl.angular.x = 0.0;
        pose_vel_ctrl.angular.y = 0.0;
        pose_vel_ctrl.angular.z = Yaw;
        pub_pose_cmd.publish(pose_vel_ctrl)
        rate.sleep()

if __name__ == '__main__':
    
    drone = str(sys.argv[1])
    X = float(sys.argv[2])
    Y = float(sys.argv[3])
    Z = float(sys.argv[4])
    Yaw = float(sys.argv[5])
    # drone=input()
    # cmd=input()
    fly2goal(drone, X,Y,Z,Yaw)