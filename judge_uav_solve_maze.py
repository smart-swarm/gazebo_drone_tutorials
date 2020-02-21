#!/usr/bin/env python
import rospy
import math
from nav_msgs.msg import Odometry
target_x = 12
target_y = 9

def pose_CB(data):
    # print(data)
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    dist = math.sqrt((x-target_x)*(x-target_x)+(y-target_y)*(y-target_y))
    z = data.pose.pose.position.z
    if (z < 5.0 and dist < 1.0):
        rospy.loginfo(rospy.get_caller_id() + " Congratulations! Solved the maze successfully.")

def judge():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('sub_pose_test', anonymous=True)
    rospy.Subscriber("/uav1/ground_truth/state", Odometry, pose_CB)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    judge()
