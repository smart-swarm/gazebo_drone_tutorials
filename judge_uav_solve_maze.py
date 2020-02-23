#!/usr/bin/env python
import rospy
import math
from nav_msgs.msg import Odometry
import time
target_x = 19
target_y = 19
start_x = -20
start_y = 19
start_time = 0

def pose_CB(data):
    # print(data)
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    dist_end = math.sqrt((x-target_x)*(x-target_x)+(y-target_y)*(y-target_y))
    dist_start = math.sqrt((x - start_x) * (x - start_x) + (y - start_y) * (y - start_y))
    z = data.pose.pose.position.z
    if (z > 1.0 && dist_start < 1.0):
        start_time = time.time()
    if (z < 5.0 and dist_end < 1.0):
        rospy.loginfo(rospy.get_caller_id() + " Congratulations! Solved the maze successfully.")
        if (start_time > 0):
            rospy.loginfo(rospy.get_caller_id() + " Time Cost: " + time.time() - start_time)

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
